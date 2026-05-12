# Quick start

## NUC side

Ships as a Docker image (libfranka 0.17.0, pinocchio 3.x, Eigen 3.4, gcc 13).

!!! warning "Host kernel must be PREEMPT_RT"

### Build & run

```bash
./fr3-stack build              # one time (~5–10 min, cached after)
./fr3-stack up cart            # cartesian impedance, anchored at current pose
./fr3-stack up hybrid --ft     # hybrid force/position with Bota FT sensor
./fr3-stack up idle            # gravity-comp + damping (hand-guidable)
./fr3-stack up cart --attach   # foreground (default is detached)
./fr3-stack logs -f
./fr3-stack down
```

Mode is `idle`, `cart`, or `hybrid`. With a non-idle initial controller, the daemon anchors the spring at the arm's startup pose — the first RT tick produces zero net force, no jump.

### Daemon flags

| Flag                     | Default      | Notes                                            |
| ------------------------ | ------------ | ------------------------------------------------ |
| `--robot <ip>`           | *required*   | FR3 IP                                           |
| `--cmd-port`             | 5555         | workstation→NUC commands                         |
| `--state-port`           | 5556         | NUC→workstation state (~200 Hz)                  |
| `--initial-controller`   | `idle`       | `idle` \| `cartesian_impedance` \| `hybrid` |
| `--ft-sensor-kind`       | empty        | currently only `bota`                            |
| `--ft-sensor-config`     | empty        | Bota: path to driver JSON                        |

Shorthands: `--idle`, `--cart`, `--hybrid`.

### Compose env vars

```bash
FR3_ROBOT_IP=192.168.1.11 \
FR3_INITIAL_CONTROLLER=cartesian_impedance \
FR3_FT_SENSOR_KIND=bota \
FR3_FT_SENSOR_CONFIG=/opt/bota/driver_config/bota_binary.json \
./fr3-stack up -d
```

### F/T sensor (optional)

The Bota EtherCAT driver is a git submodule:

```bash
git submodule update --init --recursive
./fr3-stack up hybrid -d --ft
```

When publishing, `State.wrench_ft` is a length-6 vector; otherwise `None`. `send_hybrid` refuses to start without one unless `require_ft_sensor=False`.

---

## Workstation side

```bash
pip install -e .
```

### Connection config (`fr3.yaml`)

Read from one of:

1. `$FR3_CONFIG`                   (explicit override)
2. `./fr3.yaml`                    (per-project, gitignored)
3. `~/.config/fr3/config.yaml`     (user-global)

Template (`examples/fr3.example.yaml`):

```yaml
nuc_host:  192.168.1.8       # where fr3-stack listens
robot_ip:  192.168.1.11      # FR3's own IP
desk:
  user:     dexlab
  password: thedexlab
```

Env vars override file values: `FR3_NUC_HOST`, `FR3_ROBOT_IP`, `FR3_DESK_USER`, `FR3_DESK_PASS`.

### Two API layers

| Class   | Module               | Use when                                                                        |
| ------- | -------------------- | ------------------------------------------------------------------------------- |
| `Robot` | `fr3_stack.robot`    | Streaming wire — one `send_*` per command. Direct access to every wire field.   |
| `Arm`   | `fr3_stack.client`   | Pose-centric facade for inference / teleop loops. `arm.robot` is the escape hatch. |

### Hello world (`Robot`)

```python
from fr3_stack import Robot
import numpy as np

with Robot("nuc.local") as robot:
    s = robot.wait_for_state()
    robot.send_cartesian_impedance(
        target_pos       = s.pos + np.array([0, 0, 0.05]),
        target_quat_xyzw = s.quat_xyzw,
        K                = [200, 200, 800, 30, 30, 30],
    )
    print(robot.state.pos, robot.state.wrench_ext)
```

Per-call kwargs (`K=`, `D=`, `target_wrench=`, …) update an internal last-sent cache, so subsequent calls without those kwargs reuse the new values.

### Pose-centric API (`Arm`)

Stiffness is sticky — set once, then stream `Pose` targets:

```python
from fr3_stack import Arm, Pose

with Arm("nuc.local") as arm:
    arm.set_stiffness(K=[200, 200, 800, 30, 30, 30], damp_ratio=0.9)
    arm.move_to(Pose([0.5, 0.0, 0.4], [0, 0, 0, 1]), duration=2.0)

    obs    = arm.observe()
    target = Pose(obs.pose.pos + [0, 0, 0.05], obs.pose.quat)
    arm.send(target)

    arm.hold()        # lock at current pose with cached gains
    # arm.relax()     # gravity-comp + damping (hand-guidable; sends idle)
```

`arm.move_to(target, duration=T)` blocks for `T` seconds while the daemon's min-jerk generator drives the move; afterwards the controller holds at the goal. `arm.send(...)` is fire-and-forget streaming. Hybrid commands drop through to `arm.robot.send_hybrid(...)`.

### Per-controller defaults

K/D/M defaults live in `fr3_stack/configs/<name>.yaml`, loaded when `Robot` is constructed. Profile variants (`<name>.<profile>.yaml`) let you keep multiple presets:

```python
robot = Robot("nuc.local", profiles={"cartesian_impedance": "stiff"})
robot.set_profile("hybrid", "polish")        # swap at runtime
```

### F/T calibration

If you have a sensor mounted past the wrist, run payload identification before using hybrid:

```bash
./fr3-stack up hybrid -d --ft

# manual: hand-guide to ≥6 static poses with varied EE tilt
fr3-ft-calibrate 192.168.1.8

# or record once, replay automatically (CLEAR the workspace)
fr3-ft-calibrate-record 192.168.1.8
fr3-ft-calibrate-auto   192.168.1.8
```

The solver writes `fr3_stack/sensors/bota/config/ft_calibration.yaml`, which docker-compose bind-mounts into the daemon. The daemon reads it **once at startup** — restart to pick up a new file:

```bash
./fr3-stack down
./fr3-stack up hybrid -d --ft
```

After that, `state.wrench_ft` is compensated, `state.wrench_ft_raw` carries the raw stream, and `state.ft_compensated == True`.

### Termination

```python
robot.terminate()    # sends idle with termination=True; daemon stops RT loop
```
