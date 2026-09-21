# fr3-stack

Low-latency, ROS-free control stack for the Franka Research 3.
C++ daemon on the NUC (libfranka, 1 kHz); Python client over ZMQ + Cap'n Proto.

Build the docs locally:
```bash
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs serve -f docs/config/mkdocs.yml
```
📚 [Docs]( http://127.0.0.1:8000) · [Quickstart](docs/quickstart.md)

## Install

```bash
# NUC Side
./fr3-stack build
./fr3-stack up

# Workstation
pip install -e .
```

## Hello world

```python
import time
from fr3_stack import Robot

pos, quat = [0.5, 0.0, 0.4], [0, 0, 0, 1]

with Robot("nuc.local") as robot:
    # 1. large distance movement
    robot.send_move_to(pos, quat, run_time=2.0)
    time.sleep(2.0)
    
    # 2. cartesian control for policy
    robot.send_cartesian_impedance(
        target_pos       = pos,
        target_quat_xyzw = quat,
    )

    # 3. hybrid force-position
    robot.send_hybrid_force_position(
        target_pos       = pos,
        target_quat_xyzw = quat,
        target_force     = [0, 0, -5, 0, 0, 0],
        S                = [1, 1, 0, 1, 1, 1],   # Z = force, rest = position
    )
```

Use `RobotAgent` for policy loops (`reset - observe - step`), or `Robot` for
admittance / hybrid / joint impedance.

## Controllers

| Controller | What it does |
| --- | --- |
| `idle` | hand-guidable: gravity comp + inertia-aware per-joint damping + optional Coulomb-friction comp |
| `cartesian_impedance` | spring/damper at the EE in base frame ($J^{\top}$-projected) |
| `hybrid` | per-axis force PID on `n_af` axes + position elsewhere |

Plus `MoveTo` for min-jerk setup moves.

## Dual-arm coordination

Use the [dual-arm software coordinator](docs/dual-arm.md) for paired targets
and fault handling over two independent single-arm clients. It is experimental;
real-robot evaluation and synchronized NUC execution remain pending.

## Status

Pre-alpha. APIs will change. Single FR3 only.

## License

[MIT](LICENSE)
