# third_party/bota_driver_cpp

Vendored Bota EtherCAT FT-sensor driver — C++ artifacts only. Consumed by
the daemon build via `BOTA_DRIVER_ROOT` (see top-level `CMakeLists.txt`).

```
driver_config/                 vendor JSON configs (passed via --ft-sensor-config)
linux/bota_driver_cpp_linux_x86_64/   prebuilt libBotaDriver.so + bota_driver.hpp
linux/bota_driver_cpp_linux_aarch64/  same, for ARM NUCs
LICENSE                        vendor license (see file)
```

The Python calibration / compensation tools that used to live here have
moved into the main package as `fr3_stack.sensors.bota` (mirrors
`include/fr3_stack/sensors/bota/` and `src/sensors/bota/` on the C++ side).
After `pip install -e .` (or `uv sync`) you get five console commands on
`$PATH`:

| Command                    | Purpose                                          |
| -------------------------- | ------------------------------------------------ |
| `fr3-ft-calibrate`         | manual: drive arm by hand to ≥6 poses, solve     |
| `fr3-ft-calibrate-record`  | record waypoints for the automated solver       |
| `fr3-ft-calibrate-auto`    | replay waypoints, settle at each, solve         |
| `fr3-ft-publish`           | gravity+bias-compensated wrench → CSV stdout    |
| `fr3-ft-plot`              | pipe CSV in, serve a live browser plot          |

All commands default the daemon host to `192.168.1.8` and read/write the
calibration YAML at `fr3_stack/sensors/bota/config/ft_calibration.yaml`
(in-tree; override with `--calib` or `$FR3_FT_CALIB_DIR`). The daemon
container picks up the same file via the docker-compose bind-mount onto
`/opt/fr3-stack/calib`. See `docs/quickstart.md` for the full workflow.
