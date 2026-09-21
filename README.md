# FR3-STACK

Control suite for the Franka Research 3 from DexLab. A ROS-free Python client on
your workstation connects to a real-time C++ daemon on each robot's NUC through
ZMQ. The daemon runs the controller through libfranka at 1 kHz.

[Documentation](https://robot-dexterity-lab.github.io/fr3_stack/) ·
[Quickstart](https://robot-dexterity-lab.github.io/fr3_stack/quickstart.html) ·
[v0.1.0 preview](https://github.com/Robot-Dexterity-Lab/fr3_stack/releases/tag/v0.1.0)

## Features

- Cartesian impedance, hybrid force/position control, admittance, and joint impedance.
- Pose-oriented operations with `Arm`, direct commands with `Robot`, and policy loops with `RobotAgent`.
- Bota F/T sensor integration and calibration tools.
- Experimental dual-arm coordination with one NUC per robot, paired target validation, and fault handling.

## Get started

Clone the repository on the NUC and workstation. The commands below follow the
current `main` layout.

```bash
git clone https://github.com/Robot-Dexterity-Lab/fr3_stack.git
cd fr3_stack
```

On the **NUC**, complete the real-time kernel, network, and robot setup in the
[installation guide](https://robot-dexterity-lab.github.io/fr3_stack/quickstart.html),
then build and start the daemon. `up cart` enables Cartesian impedance at the
robot's current pose.

```bash
./fr3-stack build
FR3_ROBOT_IP=192.168.1.11 ./fr3-stack up cart
# Stop the daemon:
./fr3-stack down
```

On the **workstation**, install the Python client with Python 3.10 or newer:

```bash
python3 -m pip install -e .
```

Connect to the **NUC's address**, not the robot's address. This example only reads state:

```python
from fr3_stack import Robot

with Robot("192.168.1.8") as robot:
    state = robot.wait_for_state(timeout=5.0)
    print(state.pos)        # meters, robot base frame
    print(state.quat_xyzw)  # x, y, z, w
```

See [single-arm control](https://robot-dexterity-lab.github.io/fr3_stack/single-arm.html)
for sending targets and [controllers](https://robot-dexterity-lab.github.io/fr3_stack/controllers.html)
for modes, gains, and force conventions. Closing the Python client does not stop
the daemon or clear its last command.

## Repository layout

| Path | Contents |
| --- | --- |
| `fr3_stack/` | Python clients, coordination, and sensor tools |
| `src/`, `include/` | C++ daemon and controllers |
| `proto/` | Shared Cap'n Proto schema |
| `containers/` | Dockerfile, Compose configuration, and container instructions |
| `docs/` | Website content, configuration, and documentation dependencies |
| `examples/`, `tests/` | Usage examples and automated checks |
| `AGENTS.md` | Codebase guide and verification paths for contributors and agents |

Use `./fr3-stack` from the repository root; it selects `containers/compose.yml`
and preserves the root build context and mount paths. See
[container instructions](containers/README.md) for direct Docker commands.

## Development

```bash
python3 -m pip install -e '.[dev]'
python3 -m pytest

python3 -m pip install -r docs/requirements.txt
python3 -m mkdocs serve -f docs/config/mkdocs.yml
# Check the website before publishing:
python3 -m mkdocs build --strict -f docs/config/mkdocs.yml
```

Start with [AGENTS.md](AGENTS.md) for architecture and task-specific checks.
The [development guide](https://robot-dexterity-lab.github.io/fr3_stack/development.html)
covers collaboration and publishing. Python tests use fake daemons; they do not
constitute real-robot evaluation.

## Status

`v0.1.0` is the first public preview; APIs may change. Dual-arm coordination is
experimental: real-robot evaluation is pending, and paired dispatch does not
synchronize execution on the NUCs. See the
[dual-arm guide](https://robot-dexterity-lab.github.io/fr3_stack/dual-arm.html)
for behavior and limitations.

## License

[MIT](LICENSE)
