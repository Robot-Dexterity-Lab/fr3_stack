# Multi-stage build for fr3-stack. Build context is the project root.
#
# Build:  docker build -t fr3-stack:latest .
# Run:    docker run --rm --network host \
#             --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
#             fr3-stack:latest --robot 192.168.1.11
#
# Versions are pinned to match the pixi_franka_ros2 jazzy env (so libfranka's
# pinocchio/Poco/Eigen ABI is the same one the rest of the stack uses):
#   libfranka  0.17.0
#   pinocchio  3.x      (new dep in libfranka >= 0.15; not needed by 0.13)
#   Poco       1.13.x   (Ubuntu 24.04 default; API-compatible with 1.14)
#   Eigen      3.4.x
#   gcc        13       (Ubuntu 24.04 default)
#
# Base is plain ubuntu:24.04 — no ROS 2. pinocchio 3.x comes from robotpkg
# (the upstream Debian/Ubuntu repo for stack-of-tasks software).
#
# Override libfranka version to match your robot firmware:
#   docker build --build-arg LIBFRANKA_VERSION=0.14.0 -t fr3-stack:latest .

# ---- Stage 1: build ---------------------------------------------------------

FROM ubuntu:24.04 AS builder

ARG LIBFRANKA_VERSION=0.17.0
ARG DEBIAN_FRONTEND=noninteractive

# robotpkg apt source for pinocchio 3.x. Installs under /opt/openrobots/.
# cppzmq isn't packaged on Ubuntu noble (only Debian sid has libcppzmq-dev),
# so we install it from upstream git in the next layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && install -d /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | gpg --dearmor -o /etc/apt/keyrings/robotpkg.gpg \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.gpg] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libpoco-dev \
        libeigen3-dev \
        libfmt-dev \
        libzmq3-dev \
        capnproto \
        libcapnp-dev \
        robotpkg-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# cppzmq — header-only + a small cmake config. Installs zmq.hpp and
# cppzmqConfig.cmake to /usr/local so find_package(cppzmq) works.
ARG CPPZMQ_VERSION=v4.10.0
RUN git clone --depth 1 --branch ${CPPZMQ_VERSION} \
        https://github.com/zeromq/cppzmq.git /tmp/cppzmq \
    && cmake -S /tmp/cppzmq -B /tmp/cppzmq/build \
        -DCPPZMQ_BUILD_TESTS=OFF \
    && cmake --install /tmp/cppzmq/build \
    && rm -rf /tmp/cppzmq


ENV CMAKE_PREFIX_PATH=/opt/openrobots \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig \
    LD_LIBRARY_PATH=/opt/openrobots/lib

# libfranka — built from source against the system's pinocchio/Poco/Eigen.
# CMAKE_POLICY_VERSION_MINIMUM=3.5 silences libfranka's `common/` submodule's
# old cmake_minimum_required under cmake >= 4.
RUN git clone --recursive https://github.com/frankarobotics/libfranka.git /tmp/libfranka \
    && cd /tmp/libfranka \
    && git checkout ${LIBFRANKA_VERSION} \
    && git submodule update --init --recursive \
    && cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
    && cmake --build build -j \
    && cmake --install build \
    && rm -rf /tmp/libfranka

# yaml-cpp — payload-calibration YAML loader for the FT compensation path
# (CompensatedWrenchSource). Installed AFTER libfranka so that adding /
# bumping yaml-cpp doesn't bust libfranka's ~10-min build cache. If you add
# more lightweight apt deps, put them in this same layer to keep the cache
# fault line in one place.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# fr3-stack — copy only what's needed. Schema lives at proto/fr3.capnp
# (single source of truth, shared with the Python client). Bota driver
# vendored under third_party/bota_driver_cpp/ (driver only, no payload
# compensator).
WORKDIR /src
COPY CMakeLists.txt    ./
COPY src/              ./src/
COPY include/          ./include/
COPY proto/            ./proto/
COPY third_party/      ./third_party/

# -DFOUND_LIBATOMIC=TRUE works around a bug in libcapnp-dev 1.0.1 on noble:
# its CapnProtoConfig.cmake uses check_library_exists(atomic __atomic_load_8)
# whose generated test program conflicts with gcc 13's __atomic_load_8
# built-in declaration. Pre-seeding the cache var skips the broken check.
# Tracked upstream: https://github.com/capnproto/capnproto/issues/2024
RUN cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DFOUND_LIBATOMIC=TRUE \
        -DFR3_BUILD_TEACHING=ON \
    && cmake --build build -j \
    && cmake --install build --prefix /opt/fr3-stack

# ---- Stage 2: runtime -------------------------------------------------------

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

# Use the -dev packages in runtime too — they pull in the right runtime
# .so files and the Ubuntu noble suffix for libcapnp/libfmt isn't stable
# enough to pin by name. Bloat is ~30 MB extra, traded for not breaking
# on every Ubuntu point release.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && install -d /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | gpg --dearmor -o /etc/apt/keyrings/robotpkg.gpg \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.gpg] http://robotpkg.openrobots.org/packages/debian/pub noble robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        libpoco-dev \
        libzmq3-dev \
        libcapnp-dev \
        libfmt-dev \
        libyaml-cpp0.8 \
        robotpkg-pinocchio \
    && apt-get purge -y curl gnupg \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH=/opt/openrobots/lib

# libfranka shared object — installed by the builder stage.
COPY --from=builder /usr/local/lib/libfranka.so*    /usr/local/lib/

# Bota driver — pre-built libBotaDriver.so. Run with `--cap-add=NET_RAW`
# for the EtherCAT interface, or `--device=/dev/ttyUSB0` for serial.
COPY --from=builder /src/third_party/bota_driver_cpp/linux/bota_driver_cpp_linux_x86_64/lib/libBotaDriver.so /usr/local/lib/
RUN  ldconfig

# fr3-stack binary.
COPY --from=builder /opt/fr3-stack/bin/fr3-stack     /usr/local/bin/

# fr3-ft binary — standalone Bota smoke test (no libfranka / zmq / capnp).
# Streams raw sensor-frame wrench as CSV to stdout. Used by the
# `fr3-stack-ft` compose service for verifying EtherCAT/Bota wiring without
# needing the FR3 arm online.
COPY --from=builder /opt/fr3-stack/bin/fr3-ft        /usr/local/bin/

# franka_ros2-style grav comp A/B test binary (FR3_BUILD_TEACHING=ON in
# the builder stage). Lets the operator run a literal-zero-torque
# hand-guiding loop bypassing the daemon to test whether libfranka's
# active-control API trips `joint_velocity_violation` on FR3 firmware.
COPY --from=builder /opt/fr3-stack/bin/grav_comp_franka_style  /usr/local/bin/

# Cartesian-impedance experiment harness — direct-libfranka, no ZMQ/capnp.
# Run via the `cart-test` service in docker-compose.yml. Cannot run while
# the fr3-stack daemon is up (both want the robot's FCI control connection).
COPY --from=builder /opt/fr3-stack/bin/cartesian_test           /usr/local/bin/

# Bota driver JSON configs. Pass via --bota-config-dir /opt/bota/driver_config.
COPY --from=builder /src/third_party/bota_driver_cpp/driver_config/  /opt/bota/driver_config/

EXPOSE 5555 5556

ENTRYPOINT ["fr3-stack"]
CMD ["--help"]
