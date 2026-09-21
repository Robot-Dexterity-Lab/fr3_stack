# Work with one arm

Use a Python client on your workstation to connect to the daemon running on the
NUC. The NUC maintains the robot's low-level control loop.

## Choose an interface

| Interface | Use it for |
| --- | --- |
| `Robot` | Controller parameters, state reception, and direct command streaming |
| `Arm` | Pose-oriented operations such as `observe`, `move_to`, and `hold` |
| `RobotAgent` | Policy experiments using `reset`, `observe`, and `step` |

All three use the same single-arm transport. Start with `Robot` when diagnosing
a connection, and use `Arm` when your application works primarily with poses.

## Read state first

Complete [installation and connection](quickstart.md), then run:

```python
from fr3_stack import Robot

with Robot("192.168.1.8") as robot:  # NUC address
    state = robot.wait_for_state(timeout=5.0)
    print(state.pos, state.quat_xyzw)
    print(state.running, state.last_error)
```

Positions are meters in the robot base frame. Quaternions are `[x, y, z, w]`.
`wait_for_state` waits for an initial valid state; it does not guarantee freshness
throughout a disconnected session. Check `received_at` with a local monotonic
clock when freshness matters.

## Send a Cartesian target

The following holds the received pose using Cartesian impedance. It sends a
control command; run it only with the intended robot ready for your experiment.

```python
from fr3_stack import Robot

with Robot("192.168.1.8") as robot:
    state = robot.wait_for_state()
    if not state.running or state.last_error:
        raise RuntimeError(state.last_error or "Robot control is not running")
    robot.send_cartesian_impedance(
        target_pos=state.pos,
        target_quat_xyzw=state.quat_xyzw,
    )
```

The daemon keeps the latest target until another command changes it. Closing a
Python connection does not stop the daemon or undo its last command.

## Hybrid force/position control

Use `Robot.send_hybrid_force_position` to select force or position control for
each axis. The six entries of `S` follow `[x, y, z, rx, ry, rz]`: `1` selects
position control and `0` selects force control. `target_force` is a six-component
wrench in the robot base frame, with forces in N and torques in N·m.

For a prepared contact task, this command requests +5 N along base-frame Z while
holding the other axes at the captured pose. It requires a configured, calibrated
[F/T sensor](quickstart.md#ft-sensor-optional) and sends a force-control command:

```python
# Inside an established Robot connection, with the contact setup ready:
state = robot.wait_for_state(timeout=5.0)
robot.send_hybrid_force_position(
    target_pos=state.pos,
    target_quat_xyzw=state.quat_xyzw,
    S=[1, 1, 0, 1, 1, 1],
    target_force=[0, 0, 5, 0, 0, 0],
)
```

The requested wrench is the force the robot applies to the environment; the
sensor measures the opposite reaction. Force targets on position-controlled axes
are ignored. The daemon retains the command after the client disconnects.

For a custom axis decomposition, use `Robot.send_hybrid` with `Tr` and `n_af`.
See the [hybrid controller reference](controllers.md#hybrid-forceposition) for
its inner admittance loop, outer impedance loop, gains, and force conventions.

## Work with poses

```python
from fr3_stack import Arm

with Arm("192.168.1.8") as arm:
    observation = arm.observe()
    print(observation.pose.pos)
    arm.hold()  # command a hold at the current pose
```

Use `arm.move_to(target_pose, duration=...)` for setup moves and `arm.send(pose)`
for streaming targets. Choose move duration and workspace limits for your setup;
there is no universal safe duration for every displacement or load.

## Tune the right controller

Defaults live in `fr3_stack/configs/`. Parameters supplied to `Robot.send_*` are
usually sticky: omitted values reuse the previous controller cache. Loading a
profile replaces that cache. `send_move_to` gain overrides are per-call.

Read the [controller reference](controllers.md) before choosing gains or force
modes. Hybrid control requires F/T data unless its sensor requirement is
explicitly disabled for debugging.

## Move to two arms

Keep one `Robot` connection per NUC and add the [dual-arm coordinator](dual-arm.md)
above those clients. The single-arm controller interface stays the same.
