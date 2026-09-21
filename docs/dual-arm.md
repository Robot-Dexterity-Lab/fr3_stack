# Dual-arm software coordinator

**Experimental. Real-robot evaluation has not been performed.** This implements
workstation-side orchestration, not synchronized NUC execution or a safety stop.

## Architecture

`DualArmCoordinator -> ArmEndpoint <- RobotArmEndpoint -> Robot -> NUC`

The coordinator depends on a small endpoint protocol (`snapshot`, `send`,
`stop`). It does not import Robot, ZMQ, or the wire schema. The adapter wraps an
already connected Robot. Callers own client lifecycle and provide the stop
policy; alternative transports and mock arms use the same protocol.

Use one coordinator process for the pair. A second workstation can have the
package installed, but must not send competing commands. Daemon-enforced
ownership leases are not implemented yet.

## Usage

```python
from fr3_stack import DualArmCoordinator, Robot, RobotArmEndpoint

# Define these bounded, task-specific callbacks before starting:
# stop_left(robot), stop_right(robot)
# trajectory(t) -> (left_pose_in_left_base, right_pose_in_right_base)

with Robot("left-nuc", command_send_timeout_ms=50) as left, \
     Robot("right-nuc", command_send_timeout_ms=50) as right:
    left.wait_for_state()
    right.wait_for_state()
    pair = DualArmCoordinator(
        RobotArmEndpoint(left, stop_action=stop_left),
        RobotArmEndpoint(right, stop_action=stop_right),
        max_state_age=0.1,
        max_state_skew=0.05,
    )
    pair.arm()
    pair.run(trajectory, duration=5.0, rate_hz=30.0)
```

Values above illustrate configuration, not validated operating limits.
Stop callbacks must be supplied by the application, complete promptly, and use
its approved hold/stop behavior. A no-op callback cannot stop an arm. The library
does not default to gravity compensation or guess an appropriate hold target.

## Behavior

- `arm()` checks both endpoints before entering READY. It sends no motion.
- `send(left, right)` copies, validates, and normalizes both poses before either
  send. It checks state age, reception skew, running status, and daemon errors.
- Successful pairs increment `sequence`. Sends are sequential; success means
  transport acceptance, not daemon acknowledgement or simultaneous motion.
- A detected state/transport failure latches FAULT and attempts both stop
  callbacks, even if the first raises. `fault_reason` and `stop_errors` expose
  failures. Partial transmission cannot be rolled back.
- A fault requires explicit `arm()` after resolving its cause. There is no
  automatic restart. `stop()` does not clear a pre-existing fault.
- `run()` samples both targets from one elapsed-time value, skips missed ticks
  rather than replaying a burst, and attempts both stop policies on completion,
  callback failure, or interruption. It drops a pair if its callback finishes
  after the run duration.
- Direct `send()` users must call `stop()` when finished. Closing Robot sockets
  does not stop robot motion.

State freshness uses `State.received_at`, populated at local receipt and copied
with the state snapshot. It is not serialized and is independent of the remote
robot clock. Closing a Robot invalidates its cached state. A finite command
send timeout enables ZMQ IMMEDIATE and is required by RobotArmEndpoint; the
legacy single-arm default remains unchanged.

Use the coordinator and both command sockets from one owning thread. Endpoint
calls, trajectory callbacks, and stop policies must be bounded. There is no
background watchdog: a stalled process, slow callback, or lost network can
prevent timely fault detection or stop delivery.

## What remains

No session fencing, scheduled execution, NUC-local watchdog, shared-frame
calibration, collision checking, or coordinated force controller is implemented.
Reception-time skew does not measure execution skew. Follow the
[daemon coordination plan](dual-arm-coordination-plan.md) for these extensions.

## Verification

`python3 -m pytest tests/test_dual_arm.py` uses deterministic fake endpoints and
two real ZMQ/Cap'n Proto FakeDaemons. Tests cover pair validation, partial send
failure, fault latching, independent stop attempts, stale/error states, reception
skew, loop timing, overruns, interruption, disconnect, and bounded transport sends.
No physical robot or two-NUC timing evaluation is performed by these tests.
