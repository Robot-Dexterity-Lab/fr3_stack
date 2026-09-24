# 1 kHz joint recording

Recording is off by default. Start and stop it independently while the daemon
and its controller keep running. Each active recording captures one frame per
executed libfranka callback through an SPSC ring and a non-real-time writer.
This does not change motion targets, gains, controller mode, client update rate,
or the ordinary ~200 Hz state publication.

Upgrade and rebuild the NUC daemon **and** install this version of the Python
client. Older daemons do not serve the new recording endpoint. The service uses
port **5557**, separate from motion commands (5555) and state (5556). Override
with daemon `--recording-port`, Compose `FR3_RECORDING_PORT`, and client
`recording_port` / CLI `--port`. Each arm has its own service and recording ID;
start/stop calls do not synchronize two arms.

Hardware use still requires the ordinary [setup](quickstart.md) and on-site
acceptance. Automated verification uses a synthetic producer and the actual
C++ recording service; robot timing and behavior remain unverified.

## Independent terminal commands

After installing the client with `python3 -m pip install -e .`, use the NUC's
address while its updated daemon is running:

```bash
fr3-recording --host NUC_IP start /recordings/run001.csv --recording-id run001
fr3-recording --host NUC_IP status
# After the experiment finishes:
fr3-recording --host NUC_IP stop run001
```

Every command prints JSON. Start acknowledges a successful file open and
header flush; stop acknowledges that capture is disabled and all accepted
frames have been drained, flushed and the file closed. Check `complete: true`
after stop; inspect `written`, `discarded`, `dropped`, `error`, and `error_code`.
Exit code 2 reports a refused/invalid request, I/O error, or incomplete stop;
3 means a timeout with an **unconfirmed** outcome. A successful status query
may describe an active recording; it is not proof of final completion.

The path is on the **NUC/container**, not your workstation. Parent directories
must already exist and be writable. Files are created exclusively: an existing
path is refused without truncation. Compose mounts host `./recordings` at
`/recordings`; set `FR3_RECORDINGS_DIR` on the NUC to use another host directory.
The default recordings directory is excluded from Git. Native daemon users
choose a writable absolute NUC path instead.

## Python API

```python
from fr3_stack import Robot

robot = Robot("NUC_IP")  # recording alone does not require connect()
started = robot.start_recording("/recordings/run002.csv")
print(started.recording_id)  # save in the experiment metadata
try:
    # Execute your existing experiment here.
    # Motion/state methods still need the usual robot.connect() or context manager.
    print(robot.recording_status())
finally:
    result = robot.stop_recording(started.recording_id)
    robot.close()

if not result.complete:
    raise RuntimeError(f"Incomplete recording: {result}")
```

`stop_recording()` without an ID uses this Robot object's last requested start
ID. From a new client/terminal, query status and explicitly stop the intended
ID. Closing a Python client or exiting the start CLI **does not stop recording**.
Stopping recording does not stop robot motion. Normal daemon shutdown finalizes
any active recording; an abrupt kill cannot confirm completion.

There is one active recording per daemon. A different start while busy is
refused. Repeating the current ID and path returns its current status; it never
reopens a completed run. Used IDs cannot be reused during the daemon's lifetime.
A stop with an old/different ID is refused, protecting a newer recording.
`recording_status()` describes only the current or most recent recording; save
each completed response before starting another.

All methods accept `timeout=5.0` by default. `RecordingTimeout` means the
request may already have taken effect: retain its `recording_id`, query status,
and retry the **same ID and path** if needed. Do not start another experiment
until its recording is confirmed. Refused requests and failed starts raise
`RecordingError`, whose `.status` carries the daemon result. Stop returns its
final status even on I/O failure; callers must check `.complete`.

The existing startup flag remains available for the native daemon. Relative
paths resolve against its working directory; existing files are now refused.
Use status to discover this startup recording's generated ID before stopping it.

```bash
fr3-stack --robot ROBOT_IP --joint --log-1khz /data/joint-run.csv
```

## CSV version 2

The first 45 columns retain their names, order, and meaning. Version 2 appends
35 columns, including `schema_version=2`; parse by header names instead of
assuming the old fixed width. Column suffixes `0..6` mean physical joints J1..J7.
This is the raw daemon CSV, not the proposed portable SI bundle schema.

| Fields | Meaning |
| --- | --- |
| `seq` | Executed callback counter; increments even when a full ring rejects a frame |
| `t_s` | Host steady-clock time near callback end, relative to the first logged tick |
| `robot_time_s` | Input `RobotState::time` in the robot clock; not synchronized with host time |
| `control_period_s` | Actual libfranka callback period; first callback may have period zero |
| `q0..q6`, `dq0..dq6` | Measured input state in rad and rad/s |
| `tau_J0..tau_J6` | Measured link-side sensor torque, N m |
| `tau_cmd0..tau_cmd6` | Outgoing stack command after its final slew limiter, before downstream libfranka/robot processing |
| `joint_target_valid` | 1 only for joint impedance; 0 means joint target/configuration columns are unavailable |
| `q_target0..q_target6` | Active controller target after command selection, before EMA, rad |
| `q_target_filtered0..q_target_filtered6` | Target after this callback's EMA, used for its spring term, rad |
| `K_joint0..K_joint6`, `D_joint0..D_joint6` | Applied joint stiffness and damping, N m/rad and N m s/rad |
| `filter_alpha`, `joint_use_friction` | Applied target EMA coefficient and friction-compensation flag |
| `joint_reset_count` | Joint-controller reset counter; re-entry increments it and initializes EMA from that tick's measured q |
| `controller` | Active ControllerType enum; joint impedance is 2 |

Inactive joint fields are zero with `joint_target_valid=0`; those zeros are not
usable references. Legacy `tgt_*` columns remain Cartesian targets before the
controller EMA (after dispatcher interpolation where enabled). They are zero
in joint/idle mode; do not treat them as joint targets or post-EMA Cartesian
targets. EE poses retain the base-frame and xyzw conventions.

The ordering for a joint tick is: receive state, select pending configuration
and perform any mode reset, update EMA, compute feedback/model torque, apply
the stack slew limit, then capture the frame. Capture is read-only and does not
advance the filter. Thus pre/post targets and applied gains belong to the same
computation as the recorded input state and outgoing torque.

Joint damping is `-D*dq`, not `D*(dq_ref-dq)`. To replay pre-EMA targets, apply
the recorded EMA and reset behavior; to replay post-EMA targets, bypass EMA.
Both targets are held or evolved according to the actual callback sequence,
not reconstructed from workstation send timestamps.

## I/O errors and completion

The writer checks open, header/row writes, every batch flush, final flush, and
close. Its first failing operation and errno remain available through `error()`
and `error_code()`. It reports the error immediately from the writer thread,
continues draining/discarding frames, and never stops or changes robot control
because of a disk failure.

- `ok()` is false during asynchronous startup, true after the header flush,
  and latched false after an I/O failure. It does not certify a complete run.
- `written()` counts complete rows covered by a successful `fflush`. A later
  close failure still invalidates the entire recording. This is not an fsync
  or power-loss durability guarantee.
- `discarded()` counts accepted frames without confirmed flush, including an
  unflushed prefix before a short write and frames drained after failure.
- `dropped()` counts frames rejected by the full RT ring, separately from disk
  errors. A nonzero count prevents successful completion.
- The recording manager disables capture and waits for an in-flight push on
  its non-RT service thread before calling the writer's `stop()`. The robot's
  control loop keeps running. Its permanent ring is reused only after the
  previous writer has drained and joined. `complete` requires finished,
  healthy I/O and zero drops. This certifies I/O even for a zero-frame file;
  a header-only recording is not useful experiment data.

The shutdown summary says either `I/O complete` or `INVALID recording`, with
`flushed`, `discarded`, and `dropped` counts. If any recording failed or was
incomplete, the daemon returns exit code 2 after shutting down its threads,
even if a later recording succeeded. Never fit an
invalid or partial CSV merely because some rows can be parsed. An interrupted
process without successful finalization is also not a confirmed complete log.

Successful I/O does not certify safe motion, absence of robot faults, payload
accuracy, or timing validity. Inspect actual periods, robot timestamps, mode
changes, run outcome, and metadata before selecting a fitting interval.

## Frequencies and remaining SI work

A 60 Hz client can update targets while the real joint controller runs at
1 kHz; log each control callback. Keep the original high-rate data even if a
later replay uses a coarser grid. For the planned matched replay, use a separate
1 ms calibration environment. Transfer to the current 120 Hz physics / 60 Hz
policy environment needs its own validation, including its implicit PD path.
Three milliseconds of actuator delay is not three 120 Hz simulation steps.

This logger provides targets, applied joint settings and callback timing. A
portable exporter/validator, load/tool and software provenance, final experiment
status, controller parity, independent replay/optimization, and hardware
acceptance remain separate work. It does not synchronize host/robot clocks,
identify payload, or make a failed robot run fit-ready.
