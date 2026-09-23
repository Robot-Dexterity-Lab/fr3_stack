# Hardware checklist — 1 kHz sysid log

**Branch:** `feat/sysid-1khz-ring-log`
**Written:** 2026-09-22

Everything below needs a real FR3, or at least a machine with libfranka,
cppzmq, Cap'n Proto, yaml-cpp and the Bota driver installed. None of it could
be checked on the workstation this branch was written on, so treat the feature
as **unproven on hardware** until these boxes are ticked.

## What this branch adds

A 1 kHz per-tick state log for system identification. The daemon publishes
state at ~200 Hz, which cannot resolve the few-millisecond actuation delay a
sysid fit needs, so the RT callback now records every tick into a lock-free
ring that a writer thread drains to CSV.

```bash
fr3-stack --robot <ip> --cartesian --log-1khz /data/chirp_low_fit.csv
```

Off by default. Files: `include/fr3_stack/ring_log.hpp`,
`include/fr3_stack/ring_log_writer.hpp`, `src/ring_log_writer.cpp`,
plus seven insertions in `src/main.cpp`.

## Already verified (no robot needed)

| | |
|---|---|
| Ring + writer unit tests | 45 assertions pass, zero compiler warnings |
| The exact block inserted into the RT callback | compiled and run against `tests/cpp/franka_mock`, produced one frame |
| `tests/cpp/test_controller_math.cpp` | 74 pass / 0 fail (was 72/1 on `main`; the stale hybrid test is fixed on this branch) |
| Regression guard | injecting the old absolute-velocity damping formula makes the corrected hybrid test fail, so it still guards what it was written for |

## Must be done on hardware

### 1. It compiles at all

`src/main.cpp` has **never been compiled** on this branch. The workstation has
no libfranka, so `find_package(Franka REQUIRED)` at `CMakeLists.txt:10` stops
configuration before any target is reached.

```bash
./fr3-stack build
```

Includes, declaration ordering and scoping were checked by reading, and the
inserted RT block was compiled in isolation — but the full translation unit was
not. **Do this first.** Everything below is worthless if it does not build.

Also unverified: the CMake wiring itself. `test_ring_log` was added as a target
and a ctest entry, and `src/ring_log_writer.cpp` was added to the `fr3-stack`
target. Confirm with:

```bash
cmake -S . -B build -DFR3_BUILD_TESTS=ON
cmake --build build --target test_ring_log
ctest --test-dir build --output-on-failure    # expect: controller_math, ring_log
```

### 2. The log is actually contiguous at 1 kHz

This is the whole point of the feature, and the only place it can be checked.

```bash
fr3-stack --robot <ip> --cartesian --log-1khz /tmp/idle.csv
# let it run ~60 s untouched, then Ctrl-C
```

On shutdown the daemon prints `[ring-log] wrote N frames to '...'`. Check:

- **`N ≈ 60000`** for a 60 s run. Materially fewer means ticks were missed.
- **No `WARNING: ... frame(s) dropped`.** Any drop count is a writer that
  could not keep up with the RT loop; the CSV then has gaps, visible as jumps
  in the `seq` column.
- `seq` is contiguous: `awk -F, 'NR>2 && $1 != p+1 {print NR, p, $1} {p=$1}' /tmp/idle.csv`
- `t_s` deltas are ~1 ms: check the median and the tail, not just the mean.

If frames drop on an otherwise idle NUC, raise the ring capacity (currently
16384 frames ≈ 16 s of backlog) or shorten the writer's 2 ms drain interval in
`src/ring_log_writer.cpp`.

### 3. It does not disturb the control loop

The ring was built so the RT side never blocks — `push()` is `noexcept`,
allocation-free and drops rather than waits. That is proven by unit test, not
on hardware.

- Run a motion with and without `--log-1khz` and compare. Any change in feel,
  any new audible noise, or any `communication_constraints_violation` from
  libfranka means the RT budget is being eaten and the push must be
  investigated.
- Watch for libfranka control-loop overruns in the daemon's own output.

### 4. The target columns are right

`tgt_px..tgt_qw` come from the new `Controller::pose_target()` accessor. For
`cartesian_impedance` under streaming they should track the interpolated
setpoint, not the raw last-received command.

- Stream a slow circle (`examples/03_circle.py`) with logging on.
- Plot `tgt_px` against `ee_px`: the target should lead, smoothly, with no
  step train at the client rate.
- Run `idle` and `joint_impedance`: target columns must be **zero** and the
  `controller` column must identify the mode.

### 5. Excitation and the fit itself

Only after 1–4 pass. This is block ① of the sysid port; blocks ②–④ are not
written yet.

- Write the chirp / multiband excitation client (streams
  `send_cartesian_impedance`, logs via `--log-1khz`).
- **Keep `use_friction` false during excitation.** It defaults to false for
  `cartesian_impedance`; confirm it on the wire. You are trying to *identify*
  friction, not have the controller cancel it.
- **Mount the end-effector you will actually use.** Identified armature and
  friction absorb the wrist payload. FrankaTwin's published table was fitted
  with a Franka Hand; if the target is FR3 + XHand, excite with the XHand on.
- Collect at least one **held-out** run at different gains. This is not
  optional: it is the only thing that catches the failure mode below.

### 6. The control-law parity trap

The biggest risk in the whole port, and it does not announce itself.

FrankaTwin's sim replay reproduces *its* control law. fr3_stack's
`cartesian_impedance` is not the same law — it adds a nullspace term,
joint-limit repulsion (ramp to ±10 N·m), a 1 N·m/ms torque rate limit, and
LERP/SLERP interpolation with an optional EMA. **Every τ term present on the
robot and absent from the Isaac replay gets absorbed into the fitted
friction.** The fit will converge and the numbers will look plausible.

The only defence is scoring on a held-out run at gains the fit never saw. If
held-out error is much worse than fit error, the replay is missing a term.

## Open question, not a checklist item

STEP3 of TouchEnv evaluates Isaac → MuJoCo, so the deployment chain is
Isaac → MuJoCo → real, while this work aligns Isaac ↔ real. Either MuJoCo gets
its own alignment or it stays an evaluation tool and leaves the deployment
path. Worth deciding before the fitted parameters are relied on end to end.
