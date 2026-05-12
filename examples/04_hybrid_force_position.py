"""Smoke test for hybrid force-position control (frankapy-parity API).

Three sub-demos, gated by --mode:
    press   : single-axis push (apply_effector_forces_along_axis). Press
              down on world -Z for 5 N over `--duration` seconds with a
              ramp-up / hold / ramp-down trapezoid. Safety-aborts if the
              EE drifts more than `--max-translation` from its start.

    hold    : pure-force hold (run_hybrid_force_position with target_fn=
              current pose). Selection [1,1,0,1,1,1] = Z is force-tracked,
              everything else pose-tracked. Useful for confirming the FT
              loop converges on a static target.

    streamf : streaming HFPC with a sinusoidal Z target_force overlaid on
              a held pose. Verifies the per-tick send path keeps up at
              `--dt`.

Prerequisite — daemon up with FT sensor:
        ./fr3-stack up hybrid -d --ft

    Verify wrench_ft is being published before running this script:
        python examples/00_read_state.py <nuc-host>      # has_ft must be True

Safety:
    * 3-second countdown before activation (--unsafe to skip).
    * Per-call force/torque thresholds default to 30 N translational /
      8 Nm rotational / 25 Nm joint — the daemon auto-switches to
      gravity-comp on trip and surfaces the cause via state.last_error.
    * `press` mode safety-aborts on Cartesian drift > --max-translation.

Usage:
    python examples/05_hybrid_force_position.py <nuc-host> --mode press
    python examples/05_hybrid_force_position.py <nuc-host> --mode hold --target-force-z 5
    python examples/05_hybrid_force_position.py <nuc-host> --mode streamf
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from fr3_stack import Robot


SAFE_FORCE_THRESHOLDS  = [30.0, 30.0, 30.0, 8.0, 8.0, 8.0]    # N / Nm
SAFE_TORQUE_THRESHOLDS = [15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0]  # Nm


def countdown(seconds: int, msg: str) -> None:
    for i in range(seconds, 0, -1):
        print(f"  {msg} in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 60, end="\r")


def gate_ft(robot: Robot, require_ft: bool) -> None:
    s = robot.wait_for_state()
    print(f"connected.  pos0={np.round(s.pos, 4)}  "
          f"controller={s.controller}  has_ft={s.has_ft_sensor}")
    if require_ft and not s.has_ft_sensor:
        print("\nno FT sensor frame on state.wrench_ft.")
        print("  → start the daemon with `./fr3-stack up hybrid -d --ft`,")
        print("  → or rerun with --no-ft to use libfranka's estimate.")
        sys.exit(1)


def mode_press(args, robot: Robot) -> None:
    """Single-axis force along -Z (push into the surface below the gripper)."""
    countdown(3, f"press {args.force_n} N down for {args.duration} s")
    print(f"applying [0, 0, -{args.force_n}] N for {args.duration:.1f} s "
          f"(ramp {args.acc:.2f}s, max drift {args.max_translation*1000:.0f} mm)")
    try:
        robot.apply_effector_forces_along_axis(
            run_duration       = args.duration,
            acc_duration       = args.acc,
            max_translation    = args.max_translation,
            forces             = [0.0, 0.0, -args.force_n],
            dt                 = args.dt,
            force_thresholds   = SAFE_FORCE_THRESHOLDS,
            torque_thresholds  = SAFE_TORQUE_THRESHOLDS,
            require_ft_sensor  = not args.no_ft,
        )
        print("done.")
    except RuntimeError as e:
        print(f"aborted: {e}")


def mode_hold(args, robot: Robot) -> None:
    """Pure-force hold at current pose with Z-axis force-tracked."""
    S = [1, 1, 0, 1, 1, 1]   # Z translation = force, everything else = position
    target_force = [0.0, 0.0, -args.target_force_z, 0.0, 0.0, 0.0]
    countdown(3, f"hold {target_force[2]} N on Z for {args.duration} s")
    print(f"target_force={target_force}  S={S}  duration={args.duration:.1f} s")
    robot.run_hybrid_force_position(
        duration           = args.duration,
        target_force       = target_force,
        S                  = S,
        dt                 = args.dt,
        force_thresholds   = SAFE_FORCE_THRESHOLDS,
        torque_thresholds  = SAFE_TORQUE_THRESHOLDS,
        require_ft_sensor  = not args.no_ft,
    )
    print("done.")


def mode_streamf(args, robot: Robot) -> None:
    """Streaming HFPC with a sinusoidal target_force on Z."""
    s = robot.wait_for_state()
    p0 = s.pos.copy()
    q0 = s.quat_xyzw.copy()
    S = [1, 1, 0, 1, 1, 1]
    countdown(3, f"stream Z=±{args.force_n} N sin for {args.duration} s")
    print(f"streaming Z force = {args.force_n} * sin(2π·{args.freq}·t) at "
          f"dt={args.dt}s for {args.duration:.1f} s")

    # First tick: install thresholds + S + gains.
    robot.send_hybrid_force_position(
        target_pos=p0, target_quat_xyzw=q0,
        target_force=[0, 0, 0, 0, 0, 0],
        S=S,
        force_thresholds  = SAFE_FORCE_THRESHOLDS,
        torque_thresholds = SAFE_TORQUE_THRESHOLDS,
        require_ft_sensor = not args.no_ft,
    )
    t0 = time.monotonic()
    next_t = t0
    while True:
        t = time.monotonic() - t0
        if t >= args.duration:
            break
        fz = args.force_n * math.sin(2.0 * math.pi * args.freq * t)
        robot.send_hybrid_force_position(
            target_pos=p0, target_quat_xyzw=q0,
            target_force=[0, 0, fz, 0, 0, 0],
            S=S,
            require_ft_sensor=False,
        )
        next_t += args.dt
        sleep_for = next_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.monotonic()
    print("done.")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("--mode", choices=["press", "hold", "streamf"], default="press")
    p.add_argument("--duration", type=float, default=6.0)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--force-n", dest="force_n", type=float, default=5.0,
                   help="force magnitude (N) — press/streamf modes")
    p.add_argument("--target-force-z", type=float, default=5.0,
                   help="target Z force (N) — hold mode")
    p.add_argument("--acc", type=float, default=1.0,
                   help="ramp-up/down time (s) — press mode")
    p.add_argument("--max-translation", type=float, default=0.05,
                   help="safety drift abort (m) — press mode")
    p.add_argument("--freq", type=float, default=0.25,
                   help="Hz for streamf sinusoid")
    p.add_argument("--no-ft", action="store_true")
    p.add_argument("--unsafe", action="store_true",
                   help="skip 3-second countdown")
    args = p.parse_args()

    # If --unsafe, replace countdown with a no-op.
    if args.unsafe:
        global countdown
        countdown = lambda _s, _m: None       # noqa: E731

    require_ft = not args.no_ft
    with Robot(args.host) as robot:
        gate_ft(robot, require_ft)
        try:
            if   args.mode == "press":   mode_press(args, robot)
            elif args.mode == "hold":    mode_hold(args, robot)
            elif args.mode == "streamf": mode_streamf(args, robot)
        finally:
            print("relaxing to gravity-comp ...")
            robot.send_idle()


if __name__ == "__main__":
    main()
