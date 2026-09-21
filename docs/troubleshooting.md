# Troubleshooting

Start with the failing layer: workstation connection, NUC daemon, controller,
or optional F/T sensor. Keep the reported error and active configuration together.

## No state arrives

1. Use the **NUC address** in `Robot(host)`, not the robot's FCI address.
2. Confirm the daemon is running on that NUC with `./fr3-stack ps` and inspect
   `./fr3-stack logs`.
3. Check that workstation-to-NUC traffic reaches command port 5555 and state
   port 5556, or your configured replacements.
4. Inspect daemon logs for robot connection or startup errors.

Two different NUCs can use the same port numbers. Two daemons on the same host
need different ports. For two arms, verify each connection independently first.

## A parameter change has no effect

Controller configuration searches `FR3_CONFIG_DIR`, then local `configs/`, then
packaged `fr3_stack/configs/`. Inspect the selected profile with
`robot.get_profile(controller_name)`. Named profiles replace the configuration;
they do not merge with the base YAML. Explicit `send_*` values also persist in
controller caches, so changing a YAML file does not update an existing client.

## F/T data is missing

`state.wrench_ft is None` means no complete F/T reading is available. Check the
sensor backend and configuration on the NUC. Hybrid commands normally require
this reading. A software gate bypass does not create a sensor measurement.
See [F/T setup](quickstart.md#ft-sensor-optional) and the calibration instructions.

## The dual-arm coordinator enters FAULT

Inspect `pair.fault_reason` and `pair.stop_errors`. Common causes are stale
state, excessive reception-time skew, a daemon error, or failed command delivery.
Both stop callbacks are attempted, but network loss can prevent delivery.
Resolve the cause before explicitly calling `arm()` again. See
[dual-arm behavior and limits](dual-arm.md#behavior).

## Build dependencies are missing

Use `Dockerfile` as the dependency/version reference for the NUC build. Native
builds must expose installed dependency prefixes through `CMAKE_PREFIX_PATH`.
The current CMake setup requires libfranka even when building controller mocks.
The [agent guide](https://github.com/Robot-Dexterity-Lab/fr3_stack/blob/main/AGENTS.md)
contains build and test entry points.

## A test fails to bind a local socket

Python wire tests use localhost FakeDaemons, not physical robots. A restricted
sandbox can prevent socket binding. Run them in an environment that permits
loopback networking before attributing the failure to control behavior.

## Report a reproducible problem

[Open a GitHub issue](https://github.com/Robot-Dexterity-Lab/fr3_stack/issues/new)
with the command or minimal script, commit/version, active controller/profile,
expected behavior, actual behavior, and relevant daemon error. State whether
it occurred with a FakeDaemon, one physical arm, or two physical arms. Remove
credentials from logs and configuration before sharing.
