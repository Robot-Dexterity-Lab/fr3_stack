# Hardware checklist — 1 kHz sysid log

**Integration:** [PR #5](https://github.com/Robot-Dexterity-Lab/fr3_stack/pull/5), targeting `main`
**Updated:** 2026-09-23 (CSV v2, writer failure handling, independent recording control)

The mock and I/O checks listed below have passed locally. Full daemon build
and on-robot timing/behavior still need a machine with matching dependencies
and a real FR3. Treat the feature as **unproven on hardware** until those checks
pass. See [the recording contract](docs/recording.md) for CSV and status semantics.

## What this branch adds

A 1 kHz per-tick state log for system identification. The daemon publishes
state at ~200 Hz, which cannot resolve the few-millisecond actuation delay a
sysid fit needs, so the RT callback now records every tick into a lock-free
ring that a writer thread drains to CSV.

```bash
fr3-stack --robot <ip> --joint --log-1khz /data/joint_run.csv
```

Off by default. Files: `include/fr3_stack/ring_log.hpp`,
`include/fr3_stack/ring_log_writer.hpp`, `src/ring_log_writer.cpp`,
`include/fr3_stack/ring_log_capture.hpp`, and the callback in `src/main.cpp`.
The binary startup argument is not forwarded by the container launcher. Both
native and container daemons support independent start/stop/status on port 5557:

```bash
fr3-recording --host NUC_IP start /recordings/run001.csv --recording-id run001
fr3-recording --host NUC_IP status
fr3-recording --host NUC_IP stop run001
```

Compose mounts host `FR3_RECORDINGS_DIR` (default `./recordings`) at
`/recordings`; native daemon users choose a writable absolute NUC path.
Existing files are refused without truncation. Stop finalizes only the log;
the controller keeps running. Upgrade both client and daemon before use.

## Already verified (no robot needed)

| | |
|---|---|
| Ring + writer unit tests | 105 assertions pass with `-Wall -Wextra -Werror`; short writes, header/batch/final flush, close, `/dev/full`, accounting, and 80-column CSV covered |
| Writer memory checks | AddressSanitizer + UndefinedBehaviorSanitizer: 105 pass, no findings |
| Dynamic recording lifecycle | 24 assertions pass under AddressSanitizer + UndefinedBehaviorSanitizer; in-flight stop, repeated sessions with a live producer, ID protection, exclusive files and close failures covered |
| Python/C++ recording protocol | Actual REP service compiled with `-Wall -Wextra -Werror`; Python start/stop/status, CLI, malformed/multipart requests, timeout recovery and separate motion channel covered |
| Python regression | 278 tests pass after integrating the joint excitation suite, with the C++ harness enabled; one existing Python 3.13 multiprocessing/fork deprecation warning |
| Docs and launcher | Strict MkDocs builds pass; `bash -n fr3-stack` and Compose YAML/field validation pass. Docker Compose CLI unavailable, so rendered Compose configuration and container launch remain unverified |
| Shared RT capture function | Compiled against mock states; same-tick targets/configuration, actual periods, reset/re-entry and non-joint modes covered |
| `tests/cpp/test_controller_math.cpp` | 88 pass / 0 fail; logged effective targets reproduce spring/damping torque |
| Regression guard | injecting the old absolute-velocity damping formula makes the corrected hybrid test fail, so it still guards what it was written for |

## Must be done on hardware

### 1. It compiles at all

The full `src/main.cpp` translation unit remains **unverified**. Local CMake
configuration cannot find `FrankaConfig.cmake`, so it stops before any target
is reached. Tests were compiled directly with g++; controller tests use the
libfranka mock, while the recording protocol harness uses actual Cap'n Proto,
ZMQ, the recording manager and writer with a synthetic producer.

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
cmake --build build --target test_controller_math test_ring_log test_recording recording_test_server
ctest --test-dir build --output-on-failure    # controller_math, ring_log, recording
FR3_RECORDING_TEST_SERVER="$PWD/build/recording_test_server" python3 -m pytest tests/test_recording_client.py
```

### 2. The log is actually contiguous at 1 kHz

This is the whole point of the feature, and the only place it can be checked.

```bash
fr3-stack --robot <ip> --joint --log-1khz /tmp/joint_hold.csv
# let it run ~60 s untouched, then Ctrl-C
```

On shutdown require `[ring-log] I/O complete: flushed=N discarded=0 dropped=0`.
I/O failures print their first operation/errno and an `INVALID recording`
summary; drops also make completion invalid. After normal thread shutdown an
incomplete requested log gives daemon exit code 2. Preserve this status. Check:

- **`N ≈ 60000`** for a 60 s run. Materially fewer means ticks were missed.
- **Zero dropped and discarded frames.** Any drop count is a writer that
  could not keep up with the RT loop; the CSV then has gaps, visible as jumps
  in the `seq` column.
- `seq` is contiguous: `awk -F, 'NR>2 && $1 != p+1 {print NR, p, $1} {p=$1}' /tmp/joint_hold.csv`
- Inspect `robot_time_s` differences and `control_period_s`; first period may
  be zero. Compare host callback-end `t_s` separately, without equating clocks.
  Check distribution tails and robot errors, not just mean period or row count.

If frames drop on an otherwise idle NUC, raise the ring capacity (currently
16384 frames ≈ 16 s of backlog) or shorten the writer's 2 ms drain interval in
`src/ring_log_writer.cpp`.

### 3. It does not disturb the control loop

- While holding a checked pose, start, stop and restart recording through 5557
  without restarting the daemon. Confirm controller mode, targets and gains
  remain unchanged, and state publication continues while logging is off.
- Require stop JSON `complete=true`, `discarded=0`, `dropped=0`, with `written`
  equal to the CSV's row count. Confirm each new run starts at `seq=0,t_s=0`.
- Confirm CSVs persist in the host mount after the container exits. Inspect
  status and the intended ID before stopping a recording from another terminal.

The ring was built so the RT side never blocks — `push()` is `noexcept`,
allocation-free and drops rather than waits. That is proven by unit test, not
on hardware.

- Run a motion with and without `--log-1khz` and compare. Any change in feel,
  any new audible noise, or any `communication_constraints_violation` from
  libfranka means the RT budget is being eaten and the push must be
  investigated.
- Watch for libfranka control-loop overruns in the daemon's own output.

### 4. The target columns are right

CSV v2 appends joint target and configuration columns to the original 45.
Use `joint_target_valid`, not zero/nonzero target values, to select joint rows.

- In joint mode, verify all seven `q_target*` inputs and
  `q_target_filtered*` spring targets. Recompute EMA across held and changed
  targets with the logged alpha; on a changed `joint_reset_count`, use measured
  q to initialize the filter before the update.
- Confirm logged K/D and friction flags reflect applied controller settings.
- Compare the limited outgoing `tau_cmd` with the computed torque using the
  correct slew-limit and model terms. Measured `tau_J` has different semantics.
- Outside joint mode, `joint_target_valid` must be 0 with cleared joint fields.

For optional Cartesian experiments, `tgt_px..tgt_qw` remain the active target
after dispatcher interpolation and before controller EMA. They are not the
effective spring target; replay must separately reproduce that filtering.

- Stream a slow circle (`examples/03_circle.py`) with logging on.
- Plot `tgt_px` against `ee_px`: the target should lead, smoothly, with no
  step train at the client rate.
- Run `idle` and `joint_impedance`: Cartesian `tgt_*` columns must be **zero** and the
  `controller` column must identify the mode.

### 5. Excitation and the fit itself

Only after 1-4 pass. The portable exporter/validator, payload/software/run
metadata, independent simulator replay, and optimizer are separate work.

- Use continuous `send_joint_impedance` with smooth joint sine/chirp/multisine
  references, beginning with a checked hold and small single-joint motion.
- **Keep `use_friction` false during excitation.** It defaults to false for
  joint impedance; confirm the applied flag in the CSV. You are trying to *identify*
  friction, not have the controller cancel it.
- **Mount the end-effector you will actually use.** Record its physical and
  configured load separately, including the mount. Fix and record XHand posture.
  Unknown payload must not silently be absorbed into armature or friction.
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

First verify the matched joint controller on deterministic fixtures, including
EMA, `-D*dq`, compensation, gravity conventions and limits. Then score unseen
trajectories/poses/gains. Poor held-out scores warrant investigation of model,
timing and identifiability; they do not by themselves identify a missing term.

## Open question, not a checklist item

STEP3 of TouchEnv evaluates Isaac → MuJoCo, so the deployment chain is
Isaac → MuJoCo → real, while this work aligns Isaac ↔ real. Either MuJoCo gets
its own alignment or it stays an evaluation tool and leaves the deployment
path. Worth deciding before the fitted parameters are relied on end to end.
