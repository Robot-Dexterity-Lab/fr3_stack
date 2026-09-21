# fr3-stack

Low-latency, ROS-free control stack for the Franka Research 3.

- **C++ daemon** runs on the robot's NUC, owns the libfranka connection, and ticks the chosen controller at 1 kHz.
- **Python client** (`fr3_stack`) on your workstation streams commands and reads state.

The two processes talk over ZMQ + Cap'n Proto. Both sockets use `CONFLATE=1` (latest-wins), so a slow consumer never backs up.

## Dual-arm development preview

The [software coordinator](dual-arm.md) is implemented on the
[feature branch](https://github.com/Robot-Dexterity-Lab/fr3_stack/tree/feat/dual-arm-coordinator).
**Real-robot evaluation is pending.** NUC-side synchronized execution remains
[planned work](dual-arm-coordination-plan.md). See the
[development guide](development.md) for module boundaries and verification.

## Controllers

| Name                  | Behavior                                                              |
| --------------------- | --------------------------------------------------------------------- |
| `idle`                | Gravity comp + per-joint damping. Hand-guidable; does **not** hold pose. |
| `cartesian_impedance` | 6-DoF spring/damper at the EE (Hogan 1985, $J^{\top}$-projected).     |
| `hybrid`              | Per-axis force PID on `n_af` axes + position elsewhere.               |

Plus `MoveTo` — one-shot min-jerk setup moves that ride on `cartesian_impedance`.

`hybrid` requires a calibrated F/T sensor (Bota EtherCAT supported).

## Where to start

- First time → [Quick start](quickstart.md)
- Picking a controller → [Controllers](controllers.md)
- Editing the schema → [Wire protocol](wire.md)
- How the pieces fit → [Architecture](architecture.md)
