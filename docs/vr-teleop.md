# VR teleoperation

The optional `vr-teleop` Compose profile runs the private
`ghcr.io/anthonzh/fr3-xr-teleop` image. Run this client on the Linux XR
workstation, where the headset service is available. The
FR3 daemon stays on the NUC and owns the 1 kHz control loop.

Log in to GHCR on the workstation with a GitHub token that can read the
private package (`read:packages`). The Compose profile defaults to the
validated image digest from teleop commit
`ee84044228bfe78250d22c4958ddcbe648a0891c`. Set `FR3_TELEOP_IMAGE` to
another tested digest when updating or rolling back the client.

Start the daemon on the NUC in joint-impedance mode:

```bash
./fr3-stack up joint
```

Then run the client from this checkout on the workstation:

```bash
./fr3-stack vr-teleop xhand --sim
FR3_TELEOP_HOST=nuc.local ./fr3-stack vr-teleop xhand --transmit
```

Use `plate` in place of `xhand` for the revised plate. `--sim` uses a
kinematic FR3 bridge and sends no robot commands. The physical mode requires
fresh state and the `joint_impedance` controller. Hold the right grip to move;
releasing it holds the measured joint pose. The wrapper uses `docker compose
run --no-deps`, so it does not start or restart the NUC daemon.

The stack launch uses a separate `containers/compose.vr-teleop.yml` file and
is headless. It does not mount an X11 display or GPU device; the client uses
MuJoCo only for robot kinematics. Pass `--pose-viz` after `--sim` or
`--transmit` for a lightweight MeshCat view of controller and target frames.
The XHand profile controls only the arm pose; finger motors are not driven.
Hardware behavior has not yet been validated with the lab FR3 stack.

## End-effector pose policies

Install `fr3-stack[ik]` to use SSIK's nominal FR3 solver. The optional
`Fr3PoseIK` adapter converts an absolute tool pose in `fr3_link0` to a
nearby, in-limits seven-joint target. Provide the fixed flange-to-tool
transform for a mounted plate or hand. The adapter returns `None` when no
solution is within its joint-step bound; the policy should hold and choose a
closer pose target. It does not send commands or change the daemon controller.

```python
from fr3_stack.ssik_adapter import Fr3PoseIK, fixed_tool_transform_from_urdf

flange_T_tool = fixed_tool_transform_from_urdf(robot_urdf, "right_hand_link")
ik = Fr3PoseIK(flange_T_tool=flange_T_tool)
q_target = ik.solve(base_T_tool, robot.state.q)
if q_target is not None:
    robot.send_joint_impedance(q_target)
```

The prebuilt SSIK artifact targets nominal `fr3_link8` geometry. Verify its
FK against the installed FR3 model and calibrated tool mount before use on
hardware. A calibrated arm geometry needs an SSIK artifact built from its
actual URDF and passed as `solver=`.
