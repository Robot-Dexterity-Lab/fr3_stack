"""Tune the inner cartesian-impedance gains of the admittance controller.

The admittance controller is two layers:
    outer (admittance):  M_adm·a + D_adm·v + K_adm·(inner − target) = F_ext
    inner (impedance):   τ = Jᵀ·(K·(inner − actual) − D·(J·dq)) + c

This script holds the outer loop at admittance.yaml defaults and lets you
override only the inner K / D from the command line, so you can feel the
effect of inner-loop stiffness without retuning everything.

Critical damping convention (matches admittance.yaml):
    D_t = 2·√K_t   (translational, ~141 at K_t=5000)
    D_r = 2·√K_r   (rotational,    ~14  at K_r=50)
If --D-t / --D-r are omitted they're auto-set to critical.

Usage:
    python examples/tune_inner_impedance.py <host> --K-t 8000 --K-r 80
    python examples/tune_inner_impedance.py <host> --K-t 2000          # softer
    python examples/tune_inner_impedance.py <host> --K-t 8000 --D-t 200 # over-damped
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from fr3_stack import Robot


def critical_D(K: float) -> float:
    return 2.0 * float(np.sqrt(max(K, 0.0)))


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("host", nargs="?", default="localhost")
    p.add_argument("--K-t", type=float, default=5000.0,
                   help="inner translational stiffness N/m (default 5000, yaml value)")
    p.add_argument("--K-r", type=float, default=50.0,
                   help="inner rotational stiffness Nm/rad (default 50, yaml value)")
    p.add_argument("--D-t", type=float, default=None,
                   help="inner translational damping (default: 2·√K_t, critical)")
    p.add_argument("--D-r", type=float, default=None,
                   help="inner rotational damping (default: 2·√K_r, critical)")
    p.add_argument("--hold", type=float, default=20.0,
                   help="seconds to hold (default 20)")
    p.add_argument("--rate", type=float, default=20.0,
                   help="state poll rate Hz (default 20)")
    p.add_argument("--no-ft", action="store_true",
                   help="use libfranka estimate instead of FT sensor")
    p.add_argument("--unsafe", action="store_true",
                   help="skip 3-second countdown before activation")
    args = p.parse_args()

    D_t = args.D_t if args.D_t is not None else critical_D(args.K_t)
    D_r = args.D_r if args.D_r is not None else critical_D(args.K_r)

    K_inner = [args.K_t, args.K_t, args.K_t, args.K_r, args.K_r, args.K_r]
    D_inner = [D_t,     D_t,     D_t,     D_r,     D_r,     D_r]

    require_ft = not args.no_ft

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state()
        print(f"connected.  pos0={np.round(s0.pos, 4)}  "
              f"controller={s0.controller}  has_ft={s0.has_ft_sensor}")

        if require_ft and not s0.has_ft_sensor:
            print("\nno FT sensor on state.wrench_ft — start daemon with --ft, "
                  "or pass --no-ft to fall back to libfranka's estimate.")
            return

        target_pos  = s0.pos.copy()
        target_quat = s0.quat_xyzw.copy()

        print("plan:")
        print(f"  target pose {np.round(target_pos, 4)}  quat {np.round(target_quat, 3)}")
        print(f"  outer M/K/D : admittance.yaml defaults (unchanged)")
        print(f"  inner K     : {K_inner}")
        print(f"  inner D     : {[round(x, 2) for x in D_inner]}")
        print(f"  hold {args.hold:.1f}s @ {args.rate:.0f} Hz")
        print()
        print("→ push the EE: outer admittance lets it yield; inner K decides "
              "how tightly the arm tracks the moving compliant target.")
        print("  higher K_t → 'tighter feel'; too high without enough D → ringing.")

        if not args.unsafe:
            for i in range(3, 0, -1):
                print(f"  activating in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
                time.sleep(1.0)
            print(" " * 60, end="\r")

        try:
            robot.send_admittance(
                target_pos        = target_pos,
                target_quat_xyzw  = target_quat,
                K                 = K_inner,
                D                 = D_inner,
                require_ft_sensor = require_ft,
            )
        except RuntimeError as e:
            print(f"\nactivation failed: {e}")
            return

        dt = 1.0 / args.rate
        t_end = time.monotonic() + args.hold
        peak_drift = 0.0
        try:
            while time.monotonic() < t_end:
                s = robot.state
                err   = s.pos - target_pos
                drift = float(np.linalg.norm(err))
                peak_drift = max(peak_drift, drift)

                F     = s.wrench_ft if s.has_ft_sensor else s.wrench_ext
                F_src = "ft" if s.has_ft_sensor else "lf"

                print(f"  ctrl={s.controller:12s}  "
                      f"err(mm)={np.round(err*1000, 1)}  "
                      f"|err|={drift*1000:5.1f}mm  "
                      f"F_{F_src}={np.round(F[:3], 2)}  "
                      f"M_{F_src}={np.round(F[3:], 2)}")
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\ninterrupted — sending idle ...")

        print(f"\npeak drift during test: {peak_drift*1000:.1f} mm")

        robot.send_idle()
        time.sleep(0.1)
        print("idled.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
