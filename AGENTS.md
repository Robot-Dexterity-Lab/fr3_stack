# Agent guide to fr3-stack

Use this file to orient yourself before changing the repository. It describes
how the system works, where to investigate, and how to verify a change.

## Understand the system first

fr3-stack is a ROS-free Franka Research 3 control stack with two execution sites:

- **Workstation / Python:** a user program generates targets. `Robot` sends
  single-arm commands and receives state; `Arm` adds pose-oriented operations;
  `RobotAgent` adds the policy-loop interface `reset / observe / step`.
- **NUC / C++:** `src/main.cpp` connects to one robot through libfranka, receives
  commands, runs the active controller at 1 kHz, and publishes state at ~200 Hz.
- **Wire:** Cap'n Proto messages in `proto/fr3.capnp`, carried over ZMQ. Commands
  use PUSH/PULL on port 5555; state uses PUB/SUB on port 5556. Command reception
  is latest-wins, not an acknowledged queue of motions.
- **Dual arm:** one coordinator on a workstation talks to two separate NUCs.
  Each daemon still controls one robot. `DualArmCoordinator` operates through
  `ArmEndpoint`; `RobotArmEndpoint` adapts the single-arm client. Coordination
  does not belong inside the low-level controller math.

Trace a command through `Robot.send_*` -> serialization -> `parse_command()` ->
pending command -> controller configuration -> `compute(state, model)` -> torque.
Trace feedback through daemon state publication -> `Robot._sub_loop` -> `State`.

## Choose a reading path for the task

| Task or symptom | Read these files first | Relevant tests |
| --- | --- | --- |
| Connection, missing/stale state, command delivery | `fr3_stack/robot.py`, `state.py`, `tests/conftest.py` | `tests/test_robot_client.py` |
| Pose commands or policy-loop behavior | `fr3_stack/client.py`, `agent.py`, `geometry.py`, `interpolation.py` | `test_arm.py`, `test_agent.py`, `test_geometry.py`, `test_interpolation.py` under `tests/` |
| A gain/profile does not take effect | `fr3_stack/config.py`, `configs/`, `wire/_yaml.py`, `Robot.send_*` | Profile/cache tests in `tests/test_robot_client.py` |
| Add or change a command field | `proto/fr3.capnp`, `fr3_stack/wire/`, `src/main.cpp::parse_command` | `tests/test_schema.py` and matching client tests |
| Torque, impedance, admittance, or hybrid math | `include/fr3_stack/controllers/`, `src/controllers/`, `include/fr3_stack/utils/` | `tests/cpp/test_controller_math.cpp` |
| Dual-arm validation, timing, faults | `fr3_stack/dual_arm.py`, `dual_arm_robot.py`, `docs/dual-arm.md` | `tests/test_dual_arm.py` |
| F/T readings or payload compensation | `include/fr3_stack/sensors/wrench_frame.hpp`, `src/sensors/`, `fr3_stack/sensors/bota/` | Inspect frame/calibration math; hardware behavior needs separate evaluation |
| Build, launch, or dependency failure | `CMakeLists.txt`, `Dockerfile`, `docker-compose.yml`, `fr3-stack` | Configure/build; `bash -n fr3-stack` |
| Website content/navigation | `docs/`, `mkdocs.yml`, `.github/workflows/docs.yml` | Strict MkDocs build |

## Follow the existing behavior when making a change

- Targets are meters in the arm's base frame. Quaternions on the wire are xyzw;
  Eigen constructors use wxyz. Six-vectors order translation/force before
  rotation/torque. Inspect sensor mount transforms before comparing wrenches.
- `Robot.send_*` uses per-controller sticky caches. A profile replaces its
  controller cache; `send_move_to` gain overrides are per-call. Configuration
  lookup is in `config.py`: check selected files and cache state before tuning.
- Adding a controller parameter usually touches YAML defaults, `_yaml.py`, the
  Python sender, the Cap'n Proto schema, C++ parsing, and controller config/math.
  Follow that complete path and preserve existing schema field ordinals.
- `src/main.cpp` owns controller switching, generators, target interpolation,
  and final torque-rate limiting. A test of `compute()` alone does not cover
  this dispatch logic. Keep networking and other blocking work outside RT.
- Dual-arm snapshots use local monotonic reception time. Paired targets are
  validated before sequential sends; a failure latches FAULT and attempts both
  application-supplied stop callbacks. Inspect `stop_errors`, not just exceptions.
- The current dual-arm implementation has no NUC execution synchronization,
  exclusive ownership lease, local watchdog, shared-frame calibration, or
  collision planner. `docs/dual-arm-coordination-plan.md` is planned work, not
  evidence those features exist. Real-robot evaluation remains pending.

## Practical workflow for an agent

1. Check the current branch and working-tree changes; preserve existing work.
2. Pick the reading path above. Trace the failing behavior from caller to its
   consumer and inspect a nearby test before proposing a fix.
3. Reproduce the problem with a fake daemon or deterministic math fixture where
   possible. Keep the reproducer independent of physical robot motion.
4. Change the smallest responsible layer. When crossing an interface, update
   both sides and test the contract; do not duplicate low-level logic in facades.
5. Run the relevant checks below. State what passed, what could not run, and
   what still needs hardware evaluation. Update the affected `docs/` page.

When several agents work on one task, give each a concrete file/module scope,
input/output contract, and verification command. Agree on shared interface
changes first; integrate the full path once the separate pieces are ready.

## Verification commands and limits

Python development setup and tests:

```bash
python3 -m pip install -e '.[dev]'
python3 -m pytest tests/test_dual_arm.py    # substitute the relevant test module
python3 -m pytest                         # shared API changes: full regression
```

`tests/conftest.py` provides FakeDaemon using real localhost ZMQ and Cap'n Proto.
Tests named `real_tasks` still use this fake. Socket restrictions can prevent
these tests from running; that is different from a controller failure.

C++ controller tests:

```bash
cmake -S . -B /tmp/fr3-tests -DFR3_BUILD_DAEMON=OFF -DFR3_BUILD_TESTS=ON
cmake --build /tmp/fr3-tests --target test_controller_math -j2
ctest --test-dir /tmp/fr3-tests --output-on-failure
```

The current CMake configuration still calls `find_package(Franka REQUIRED)` even
for mock tests. Provide installed dependency prefixes through `CMAKE_PREFIX_PATH`
when necessary; the mock test executable itself uses `tests/cpp/franka_mock/`.
The full daemon additionally needs cppzmq, Cap'n Proto, yaml-cpp, and the Bota
library; use `Dockerfile` to identify the intended dependency versions.

Documentation:

```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs build --strict
```

`docs/` is the GitHub Pages source; `site/` is generated output. Do not create a
second copy of website content. Consult `docs/development.md` for deployment.
`examples/`, daemon launch commands, and calibration tools can operate hardware;
use them only as part of an explicitly requested hardware task, not unit tests.
