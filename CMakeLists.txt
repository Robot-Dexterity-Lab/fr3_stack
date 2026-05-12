cmake_minimum_required(VERSION 3.15)
project(fr3-stack LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Required for everything below: controllers static lib, daemon, teaching
# binaries, math-mock tests.
find_package(Franka  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Threads REQUIRED)

# Static lib of all controller implementations. Both the daemon
# (src/main.cpp) and the standalone demo binaries (src/bin/) link to it,
# so there's exactly one place each controller's math gets compiled.
# (Tests recompile the same .cpps separately against the franka mock —
# see the FR3_BUILD_TESTS section below.)
add_library(fr3_controllers STATIC
    src/controllers/gravity_compensation_controller.cpp
    src/controllers/cartesian_impedance_controller.cpp
    src/controllers/joint_impedance_controller.cpp
    src/controllers/cartesian_admittance_controller.cpp
    src/controllers/hybrid_force_motion_controller.cpp)
target_include_directories(fr3_controllers PUBLIC
    ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(fr3_controllers
    PUBLIC Franka::Franka Eigen3::Eigen)
target_compile_options(fr3_controllers PRIVATE -Wall -Wextra -O2)

# fr3-stack daemon. Pulls in ZMQ + capnp + the bota FT driver — extra deps
# the standalone teaching binaries / math-mock tests don't need, so they
# sit behind FR3_BUILD_DAEMON. Defaults to ON: production / Docker builds
# behave the same as before. Turn OFF to build just the controller lib +
# teaching binaries on a machine without zmq/capnp.
option(FR3_BUILD_DAEMON "Build the fr3-stack daemon" ON)
if(FR3_BUILD_DAEMON)
    find_package(cppzmq    REQUIRED)        # apt: libzmq3-dev libcppzmq-dev
    find_package(CapnProto REQUIRED)        # apt: capnproto libcapnp-dev
    find_package(yaml-cpp  REQUIRED)        # apt: libyaml-cpp-dev — payload calib YAML

    # Generate C++ from the shared .capnp schema. Single source of truth at
    # proto/fr3.capnp — Python loads it via importlib/walk-up; C++ generates
    # fr3.capnp.{h,c++} from it here. CAPNPC_SRC_PREFIX lets the schema live
    # outside this CMake source dir.
    set(CAPNPC_SRC_PREFIX ${CMAKE_SOURCE_DIR}/proto)
    set(CAPNP_SRC ${CMAKE_SOURCE_DIR}/proto/fr3.capnp)
    capnp_generate_cpp(CAPNP_GEN_SRCS CAPNP_GEN_HDRS ${CAPNP_SRC})

    # Bota FT-sensor driver: vendored prebuilt .so + headers expected at
    # ${BOTA_DRIVER_ROOT} (override via -DBOTA_DRIVER_ROOT=... on configure).
    # To add ATI / Robotiq / etc.:
    #   1) write src/sensors/<vendor>/<vendor>_wrench_source.cpp + matching .hpp,
    #   2) register the kind string in src/sensors/bota/wrench_source.cpp,
    #   3) add the .cpp to fr3-stack sources below and link any vendor library.
    set(BOTA_DRIVER_ROOT
        "${CMAKE_SOURCE_DIR}/third_party/bota_driver_cpp/linux/bota_driver_cpp_linux_x86_64"
        CACHE PATH "Path to vendored bota_driver_cpp linux release")

    add_executable(fr3-stack
        src/main.cpp
        src/sensors/bota/wrench_source.cpp
        src/sensors/bota/bota_wrench_source.cpp
        src/sensors/compensated_wrench_source.cpp
        src/sensors/payload_calib.cpp
        ${CAPNP_GEN_SRCS}
        ${CAPNP_GEN_HDRS})  # listed so parallel builds wait for header gen
    target_include_directories(fr3-stack PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}          # generated headers root
        ${CMAKE_CURRENT_BINARY_DIR}/proto    # for the generated fr3.capnp.h
        ${BOTA_DRIVER_ROOT}/include)
    target_link_libraries(fr3-stack
        PRIVATE
            fr3_controllers                    # pulls Franka + Eigen transitively
            Threads::Threads
            cppzmq
            CapnProto::capnp
            yaml-cpp
            ${BOTA_DRIVER_ROOT}/lib/libBotaDriver.so)
    target_compile_options(fr3-stack PRIVATE -Wall -Wextra -O2)

    install(TARGETS fr3-stack RUNTIME DESTINATION bin)

    # fr3-ft — standalone Bota smoke-test daemon. No libfranka, no zmq, no
    # capnp; just opens BotaWrenchSource and prints CSV to stdout. Bundled
    # under FR3_BUILD_DAEMON because it needs the same vendored Bota driver.
    add_executable(fr3-ft
        src/bin/ft_dump.cpp
        src/sensors/bota/wrench_source.cpp
        src/sensors/bota/bota_wrench_source.cpp)
    target_include_directories(fr3-ft PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${BOTA_DRIVER_ROOT}/include)
    target_link_libraries(fr3-ft
        PRIVATE
            Eigen3::Eigen
            Threads::Threads
            ${BOTA_DRIVER_ROOT}/lib/libBotaDriver.so)
    target_compile_options(fr3-ft PRIVATE -Wall -Wextra -O2)
    install(TARGETS fr3-ft RUNTIME DESTINATION bin)
endif()

# Standalone demo binaries under src/bin/. Each has its own main() and
# wraps a single controller from fr3_controllers with a local demo
# updater (no ZMQ / capnp / sensors). Build with -DFR3_BUILD_TEACHING=ON;
# off by default so production builds stay lean.
option(FR3_BUILD_TEACHING "Build standalone demo binaries" OFF)
if(FR3_BUILD_TEACHING)
    foreach(_bin cartesian_impedance cartesian_test joint_impedance hybrid_force_motion)
        add_executable(${_bin} src/bin/${_bin}.cpp)
        target_link_libraries(${_bin} PRIVATE fr3_controllers Threads::Threads)
        target_compile_options(${_bin} PRIVATE -Wall -Wextra -O2)
        install(TARGETS ${_bin} RUNTIME DESTINATION bin)
    endforeach()

    # franka_ros2-style grav comp A/B test: literal zero torque +
    # active-control API + no setCollisionBehavior. Doesn't depend on
    # fr3_controllers. See src/bin/grav_comp_franka_style.cpp.
    add_executable(grav_comp_franka_style src/bin/grav_comp_franka_style.cpp)
    target_link_libraries(grav_comp_franka_style
        PRIVATE Franka::Franka Threads::Threads)
    target_compile_options(grav_comp_franka_style
        PRIVATE -Wall -Wextra -O2)
    install(TARGETS grav_comp_franka_style RUNTIME DESTINATION bin)
endif()

# Math-mock tests for the controllers + utilities. Uses a tiny libfranka shim
# under tests/cpp/franka_mock/ so the test compiles on dev machines that
# don't have libfranka (macOS, CI without RT kernel). The controller .cpps
# are compiled INTO the test target (not linked from fr3_controllers) so
# the franka_mock include path takes precedence over the real headers.
# Run:
#     cmake -B build -DFR3_BUILD_TESTS=ON
#     cmake --build build --target test_controller_math
#     ./build/test_controller_math
option(FR3_BUILD_TESTS "Build math-mock unit tests" OFF)
if(FR3_BUILD_TESTS)
    add_executable(test_controller_math
        tests/cpp/test_controller_math.cpp
        src/controllers/gravity_compensation_controller.cpp
        src/controllers/cartesian_impedance_controller.cpp
        src/controllers/joint_impedance_controller.cpp
        src/controllers/cartesian_admittance_controller.cpp
        src/controllers/hybrid_force_motion_controller.cpp)
    target_include_directories(test_controller_math PRIVATE
        ${CMAKE_SOURCE_DIR}/tests/cpp/franka_mock   # franka mock first
        ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(test_controller_math PRIVATE Eigen3::Eigen)
    target_compile_options(test_controller_math
        PRIVATE -Wall -Wextra -Wno-unused-function -O2)

    enable_testing()
    add_test(NAME controller_math COMMAND test_controller_math)
endif()
