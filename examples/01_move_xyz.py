"""Simple Cartesian-impedance API example, with safety guards.

Reads current pose, moves the EE by (dx, dy, dz) meters in the base frame,
holds for `--hold` seconds, then releases to idle (gravity comp).

Safety defaults (override with `--unsafe`):
  - per-axis delta clamped to ±10 cm
  - low stiffness (K_xy=150, K_z=300, K_rot=15) so a wrong target is gentle
  - 3-second countdown after printing the plan, abortable with Ctrl+C

Usage:
    python examples/01_move_xyz.py <nuc-host> [dx dy dz] [--hold S]

The host is the NUC running fr3-stack (e.g. 192.168.1.8), NOT the robot's IP.

Examples:
    python examples/01_move_xyz.py 192.168.1.8 0 0 0.05         # 5 cm up
    python examples/01_move_xyz.py 192.168.1.8 0.05 0 0 --hold 5
    python examples/01_move_xyz.py 192.168.1.8                  # interactive
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from fr3_stack import Robot


SAFE_MAX_DELTA = 0.10            # ±10 cm per axis
SAFE_K_XY      = 150.0
SAFE_K_Z       = 300.0
SAFE_K_ROT     = 15.0


def countdown(seconds: int, msg: str) -> None:
    for i in range(seconds, 0, -1):
        print(f"  {msg} in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 60, end="\r")     # clear the line


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("delta", nargs="*", type=float,
                   help="dx dy dz in meters (omit for interactive prompt)")
    p.add_argument("--hold", type=float, default=3.0,
                   help="seconds to hold the new target before idling")
    p.add_argument("--rate", type=float, default=100.0,
                   help="streaming rate Hz")
    p.add_argument("--K-xy",  type=float, default=SAFE_K_XY)
    p.add_argument("--K-z",   type=float, default=SAFE_K_Z)
    p.add_argument("--K-rot", type=float, default=SAFE_K_ROT)
    p.add_argument("--unsafe", action="store_true",
                   help="bypass delta clamp and confirmation countdown")
    args = p.parse_args()

    if args.delta and len(args.delta) != 3:
        p.error("delta needs exactly 3 numbers (dx dy dz)")

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state()
        print(f"connected.  pos0 = {np.round(s0.pos, 4)}  controller={s0.controller}")

        if args.delta:
            dx, dy, dz = args.delta
        else:
            line = input("dx dy dz (meters, e.g. '0 0 0.05'): ")
            dx, dy, dz = (float(x) for x in line.split())

        # Safety clamp.
        if not args.unsafe:
            clipped = [max(-SAFE_MAX_DELTA, min(SAFE_MAX_DELTA, d))
                       for d in (dx, dy, dz)]
            if clipped != [dx, dy, dz]:
                print(f"⚠️  delta clamped to ±{SAFE_MAX_DELTA*100:.0f} cm: "
                      f"{(dx, dy, dz)} → {tuple(clipped)} "
                      f"(use --unsafe to bypass)")
                dx, dy, dz = clipped

        target_pos = s0.pos + np.array([dx, dy, dz])
        K = np.array([args.K_xy, args.K_xy, args.K_z,
                      args.K_rot, args.K_rot, args.K_rot])

        print(f"plan:")
        print(f"  from   {np.round(s0.pos, 4)}")
        print(f"  to     {np.round(target_pos, 4)}  "
              f"(Δ {dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m")
        print(f"  K      xy={args.K_xy:.0f}  z={args.K_z:.0f}  rot={args.K_rot:.0f}")
        print(f"  hold   {args.hold:.1f} s @ {args.rate:.0f} Hz")

        if not args.unsafe:
            try:
                countdown(3, "starting")
            except KeyboardInterrupt:
                print("\nabort. arm untouched.")
                return

        # Stream the same target at `rate` Hz for `hold` seconds. The C++
        # side smooths it (1st-order LP, filter_alpha=0.05 default), so the
        # arm slews instead of jumping.
        dt = 1.0 / args.rate
        t_end = time.monotonic() + args.hold
        try:
            while time.monotonic() < t_end:
                robot.send_cartesian_impedance(
                    target_pos       = target_pos,
                    target_quat_xyzw = s0.quat_xyzw,   # keep start orientation
                    K                = K,
                )

                if int((t_end - time.monotonic()) * 2) % 2 == 0:
                    s = robot.state
                    err = s.pos - target_pos
                    print(f"  ctrl={s.controller:20s}  "
                          f"pos={np.round(s.pos, 4)}  "
                          f"err={np.round(err, 4)}  "
                          f"F_ext={np.round(s.wrench_ext[:3], 2)}")

                time.sleep(dt)
        except KeyboardInterrupt:
            print("\ninterrupted — sending idle ...")

        # Hand the arm back to gravity-comp before disconnecting.
        robot.send_idle()
        time.sleep(0.1)
        print("idled.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
