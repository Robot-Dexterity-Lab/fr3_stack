# Dual-arm coordination implementation plan

**Status: proposed daemon-side extension.** The [software coordinator](dual-arm.md)
implements paired validation, a common workstation loop, and fault propagation.
The leases, scheduled execution, local watchdogs, and clock synchronization below
are not implemented. **No real-robot evaluation has been performed.**

Repository: [Robot-Dexterity-Lab/fr3_stack](https://github.com/Robot-Dexterity-Lab/fr3_stack).

## Deployment and scope

Two FR3 arms each connect to a dedicated NUC running a daemon. Both workstation
computers may have the Python client installed, but only one workstation owns
the active dual-arm control session at a time. That coordinator connects to
both NUCs using distinct addresses; each NUC can use ports 5555/5556.

The first milestone is paired trajectory execution with bounded timing,
readiness checks, and independent local watchdogs. Shared-object force control,
collision avoidance, and hardware safety interlocking are separate work.

Current `Robot` instances provide independent command and state connections.
The daemon accepts commands without session ownership or execution timestamps.
Latest-wins streaming alone does not implement dual-arm coordination.

## Implementation steps

### 1. Session ownership and paired API

Add a `DualArmCoordinator` alongside `fr3_stack/robot.py`. It owns two clients,
validates both targets before dispatch, and exposes connect, observe-pair,
prepare, start, hold, and close operations. Configuration names each arm,
its NUC endpoints, calibration, and timing limits explicitly.

Each NUC grants an expiring exclusive control lease identified by a session
ID and fencing epoch. Reject commands from other owners and older epochs.
Acquiring only one arm never enables execution; release it if the second
acquisition fails. Lease expiry requires explicit re-arming, not automatic
resumption. While leased, reject legacy motion commands that bypass ownership.
Keep monitoring available independently of command ownership.

### 2. Acknowledged trajectory protocol

Extend `proto/fr3.capnp` without changing existing field ordinals. Add a
separate non-conflating coordination channel with explicit acknowledgements,
bounded retries, deduplication, and bounded buffers. Preserve the existing
single-arm API outside coordinated sessions.

A paired segment carries session/epoch, sequence number, content identifier,
start time, duration, expiry, and the target trajectory for each arm. Both NUCs
validate and buffer PREPARE, then acknowledge readiness for that exact segment.
The coordinator sends COMMIT only after both are ready and before a defined
commit deadline. Each daemon authorizes execution only for a matching prepared
segment received within its deadline. Duplicate messages are idempotent;
conflicting duplicates, out-of-order messages, and expired segments are rejected.

A timeout before commit abandons the segment. A missing commit acknowledgement
makes the outcome uncertain: request HOLD on both arms and enter FAULT rather
than assuming neither arm moved. Network partitions can deliver commit to only
one arm; this protocol is not an atomic physical-start guarantee.

### 3. Clock and execution scheduling

Synchronize NUC clocks, preferably using PTP where supported, and measure the
actual synchronization error. Define a shared execution time domain and map
scheduled starts to local monotonic deadlines. Use monotonic time for lease
and watchdog expiry. Detect clock steps and invalidate pending schedules.

Before arming, require acceptable clock uncertainty, fresh states from both
arms, and sufficient scheduling lead time. Configure these limits explicitly;
choose acceptance thresholds from measurements before physical evaluation.
Interpolate buffered trajectories locally at 1 kHz. Parsing, networking,
allocation, and acknowledgement handling stay outside the RT callback.

### 4. Coordinated faults and local watchdogs

Use `DISCONNECTED -> READY -> ARMED -> RUNNING`, with `HOLD` and `FAULT` exits.
A stale state, expired lease, missed deadline, exhausted trajectory buffer,
or fault on either arm stops admission of new paired work. The coordinator
requests HOLD on both and reports each arm's last confirmed execution state.
Each NUC independently enforces lease, communication, and buffer watchdogs even
when the workstation is unreachable. Buffered future segments are invalidated
on a fault or session change.

HOLD must be a defined, bounded transition from the current commanded motion,
not an abrupt target jump or an automatic switch to gravity compensation.
Select and evaluate the transition with task-specific limits. A hardware fault
may prevent an arm from holding; report that failure instead of claiming a
successful paired stop. Recovery requires explicit acknowledgement and re-arm.

Software cannot guarantee simultaneous stopping during a network partition.
Tasks needing a hard coupled stop require an appropriate hardware safety path.

### 5. Shared coordinates

Store versioned transforms `T_world_base_left` and `T_world_base_right`.
Specify paired end-effector trajectories in a common workcell frame and convert
with `T_base_ee = inverse(T_world_base) * T_world_ee`. Bind calibration versions
to the prepared session so they cannot change mid-trajectory. Preserve xyzw
wire quaternion ordering. Validate transforms and workspace limits before arm.

Shared coordinates alone do not provide inter-arm collision checking or
compliant shared-object manipulation.

## Verification and acceptance gates

All items below are planned; none are claimed complete by this document.

| Gate | Checks | Required evidence |
| --- | --- | --- |
| Offline unit tests | Frame transforms, session fencing, state transitions, target validation, queue bounds | Deterministic pass/fail results |
| Two simulated daemons | Isolated commands/states; dropped, delayed, reordered and duplicate messages; one-sided prepare/commit; restart; competing workstation; clock steps | Neither daemon executes unprepared or expired work; local watchdogs act without coordinator recovery |
| Two NUCs without actuation | Timing and scheduling under load; disconnects and coordinator crashes | Clock uncertainty, start skew, deadline misses, hold detection latency, RT overruns |
| Supervised low-speed robot evaluation | Separate workspaces, paired start/hold, approved fault scenarios | Recorded trajectories and fault response against predeclared limits |
| Task-specific evaluation | Relative-pose error, clearance, contact forces where applicable | Acceptance criteria for the intended coordinated task |

For every segment, log session/epoch, sequence, prepare/commit acknowledgements,
scheduled and observed start times, clock uncertainty, state ages, and stop
reason. Logs must identify uncertainty rather than infer successful execution
from command transmission. Keep logging outside RT.

**Real evaluation remains pending.** Before that gate, establish numeric limits
for execution skew, state age, scheduling lead time, watchdog detection, and
hold behavior. Mock tests cannot establish hardware stability or safety.

## Deliverables and implementation order

1. Protocol specification, session ownership, and deterministic state-machine tests.
2. Coordinator API plus two-daemon fault-injection tests.
3. NUC scheduling, bounded buffers, watchdogs, and timing instrumentation.
4. Shared-frame configuration and a non-actuating two-NUC diagnostic example.
5. English operator instructions and a separate real-evaluation report once run.

Keep these as small reviewable changes. Do not enable coordinated hardware
motion by default or represent workstation-side dispatch as synchronized execution.
