# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

**Every change to this repository gets an entry here.** See the changelog rule
in [AGENTS.md](AGENTS.md).

## [Unreleased]

### Added
- Independent `Robot.start_recording` / `stop_recording` / `recording_status`
  and `fr3-recording` CLI, with acknowledged Cap'n Proto REQ/REP on port 5557
  (`--recording-port`). Recording can be toggled without daemon restart or
  controller changes; stop drains and closes before replying. Per-run IDs
  support retries and reject stale stops. New files are created exclusively,
  including `--log-1khz`, so an existing recording is never overwritten.
  `containers/compose.yml` exposes `FR3_RECORDING_PORT` and mounts
  `FR3_RECORDINGS_DIR` (default `./recordings`, ignored by Git) at `/recordings`.
  Both daemon and Python client need updating. See `docs/recording.md`;
  concurrent lifecycle, I/O failures, and actual Python/C++ wire tests cover
  software behavior, with NUC timing/hardware acceptance still pending.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- CSV v2 in `src/ring_log_writer.cpp` appends same-tick pre/post-EMA joint
  targets, applied K/D/filter/friction settings, reset count, robot time and
  actual callback period. `ring_log_capture.hpp` shares the daemon capture path
  with mock tests; inactive joint fields carry an explicit validity flag.
  Existing CSV columns keep their positions; fixed-width consumers must accept
  the new 80-column schema. See `docs/recording.md` and
  [#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5).
- `TODO_FRANKATWIN.md` and `docs/joint-sysid.md` now document the integrated
  CSV v2/writer fixes and independent recording start/stop/status
  (Python/CLI, port 5557), with links to `docs/recording.md`, separately from remaining
  daemon build, deployment, hardware acceptance, and portable-export work.
  Existing client diagnostics remain non-fit-ready. Related work:
  [#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5).
- `examples/joint_sysid.py` and `docs/joint-sysid.md`: offline CSV/SVG previews
  and opt-in single-arm joint hold/sine/chirp/multisine streaming from the live
  pose. Includes smooth envelopes, joint/reference limits, feedback/timing
  guards, explicit gain/filter recording, partial-run status, and best-effort
  termination on faults. Client logs are explicitly distinguished from the
  pending 1 kHz identification contract; no daemon or sim dependency changes.
  Adds deterministic trajectory/failure tests and a localhost FakeDaemon wire
  test. Related logging work: [#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5).
- `TODO_FRANKATWIN.md`: local checklist for fixing sysid log error reporting,
  defining target replay semantics, validating NUC data collection, and
  exporting logs for independent simulation identification. Prioritizes joint-target
  logging and staged joint chirp/multisine experiments with held-out validation,
  and records the unresolved mount/XHand payload. Simulation exchanges files
  without a runtime dependency on this stack. Separates verified mock
  tests from pending hardware work so experiments can build on the existing
  logging branch. Related work: [#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5).
- `docs/joint-sysid.md`, `TODO_FRANKATWIN.md`, and README links now document
  the adopted joint-space simulation parameter identification route. Separate
  the existing continuous joint interface and implemented logging from pending SI work,
  and define payload provenance, portable-file handoff, controller/timing
  parity, staged fitting, independent validation, profile export, and training
  transfer gates. The sim repository remains independent; planned interfaces
  are not advertised as implemented. Related logging work:
  [#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5).
- Optional 1 kHz state log for system identification, off by default
  (`--log-1khz <path.csv>`). The daemon runs its controller at 1 kHz but
  publishes state at ~200 Hz, which cannot resolve the few-millisecond
  actuation delay a sysid fit needs. The RT callback now records every tick
  into a lock-free SPSC ring (`include/fr3_stack/ring_log.hpp`) that a writer
  thread drains to CSV (`src/ring_log_writer.cpp`). `push()` is `noexcept`,
  allocation-free and never waits on the consumer: when the writer falls
  behind it drops the arriving frame and counts it, so a log states where its
  gaps are (jumps in the `seq` column) instead of stalling the control loop.
  Frames carry the commanded target and the post-rate-limit torque alongside
  the measured state, because identification replays the commanded trajectory
  in simulation. ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- `Controller::pose_target()`, a read-only accessor for the SE(3) setpoint a
  controller is currently tracking. Defaults to `false` for controllers that
  track no pose; overridden by Cartesian impedance, admittance and hybrid.
  Added so the ring log reads the live setpoint rather than reconstructing the
  generator / interpolator / raw-cfg precedence in `main.cpp`.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- `tests/cpp/test_ring_log.cpp`: 45 assertions covering ring semantics, FIFO
  order under concurrent single-producer/single-consumer use, gap accounting,
  the CSV round trip and an unwritable log path. Wired into CMake as the
  `test_ring_log` target and a `ring_log` ctest entry.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- `HARDWARE_CHECKLIST.md`: what still needs a real robot before the 1 kHz log
  can be trusted — that it compiles at all, that the log is contiguous with
  zero drops, that it does not disturb the control loop, that the target
  columns track the interpolated setpoint, and the control-law parity trap
  that silently corrupts a fit.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- `CHANGELOG.md`, this file, reconstructed for 0.1.0 from the repository's own
  history and README. `AGENTS.md` now requires an entry under `[Unreleased]`
  for every change, in the commit or PR that makes it, and the README's Status
  section links here.
- `CITATION.cff` and a Citation section in the README, so the v0.1.0 preview
  is citable and GitHub renders *Cite this repository* in the sidebar.
  ([#6](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/6))
- `tau_J` in the libfranka mock (`tests/cpp/franka_mock/`). Real libfranka has
  it and the mock did not, so the mock could not compile the code the daemon
  now runs. ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))

### Fixed
- `RingLogWriter` now detects short writes and header/row/final-flush/close
  failures, retains the first operation/errno, and counts only flushed rows as
  written. Failed streams keep draining off the RT thread. Completion requires
  successful finalization and zero ring drops; `--log-1khz` returns exit code 2
  after normal daemon shutdown if logging is incomplete. Deterministic failure
  fixtures and `/dev/full` prevent disk errors from masquerading as usable SI
  data. Hardware timing and experiment validation remain required.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))
- `test_hybrid_outer_damp_uses_error_velocity` had been failing on `main`. Its
  expected window came from an analytic trace written for
  `CartesianAdmittance` and applied unchanged to `HybridForceMotion`, but
  hybrid damps against a low-passed inner velocity (`inner_v_filter_alpha`,
  0.1 by default) to avoid a ~100 Hz buzz from the LERP'd target's velocity
  discontinuities. On tick 1 that gives `200·1e-6 + 28·1e-4 = 3.0e-3`, which
  is what the controller has always produced. **No torque math changed** — the
  controller was correct and the expectation was not. The corrected test keeps
  the original guarantee and adds a pass-through case (`alpha = 1.0`) where
  hybrid must reproduce the admittance number exactly, so the two outer-damping
  formulas cannot drift apart unnoticed.
  ([#5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5))

## [0.1.0] — 2026-09-21

First public preview. APIs may change.

### Added
- ROS-free control stack for the Franka Research 3: a Python client on the
  workstation talking to a real-time C++ daemon on each robot's NUC over ZMQ,
  with Cap'n Proto messages (`proto/fr3.capnp`). Commands use PUSH/PULL on port
  5555, state uses PUB/SUB on 5556. The daemon runs the active controller
  through libfranka at 1 kHz and publishes state at ~200 Hz.
- Controllers: `idle` (hand-guidable gravity compensation with inertia-aware
  per-joint damping and optional Coulomb-friction compensation),
  `cartesian_impedance`, `hybrid` force/position (layered HFVC inner loop with
  per-axis force PID over an outer impedance loop), `admittance`, and
  `joint_impedance`. Joint-limit repulsion, Stribeck-style friction
  compensation and a 1 N·m/ms torque rate limit are summed into τ regardless
  of the active controller.
- `MoveTo`: one-shot min-jerk setup moves, run by a trajectory generator that
  overrides the pose target each tick and is then dropped.
- Streaming target interpolation bridging any client rate from 5 Hz to 1 kHz
  by LERP/SLERP between the two most recent received targets, plus a
  first-order low-pass (`filter_alpha`). `fr3_stack.InterpolationController`
  handles sparse policy chunks client-side.
- Python API layers: `Robot` for direct commands, `Arm` for pose-oriented
  operations, `RobotAgent` for policy loops (`reset` / `observe` / `step`).
  Per-controller sticky caches and runtime-swappable YAML profiles
  (`fr3_stack/configs/`).
- Bota F/T sensor integration with payload calibration and a compensated
  wrench source; both compensated and raw wrenches are published.
- Experimental dual-arm coordination: one coordinator on a workstation driving
  two NUCs through `ArmEndpoint`, with paired target validation and fault
  handling. No NUC execution synchronization, ownership lease, watchdog,
  shared-frame calibration or collision planner; real-robot evaluation pending.
  ([#3](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/3))
- Container configuration under `containers/` and the `./fr3-stack` launcher.
- Documentation site under `docs/`, published to GitHub Pages.
- `AGENTS.md`, a practical project guide for contributors and agents.
  ([#4](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/4))
- Math-mock C++ tests (`tests/cpp/test_controller_math.cpp`) against a small
  libfranka mock, and Python tests using a fake daemon over real localhost ZMQ.

[Unreleased]: https://github.com/Robot-Dexterity-Lab/fr3_stack/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/Robot-Dexterity-Lab/fr3_stack/releases/tag/v0.1.0
