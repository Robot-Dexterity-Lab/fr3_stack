"""Quick parameterized hybrid circle for tuning iteration.

Streams an XY circle with a constant Z press force through
``run_hybrid_force_position``. Activates whatever hybrid profile you name
(``--profile circle`` for the tuned one) and reports radial tracking error
at the end so you can A/B different profiles / params without re-typing
the boilerplate each time.

Examples:
    # current tuned circle profile
    python examples/circle_quick.py 192.168.1.8 --profile circle

    # baseline (no profile, uses hybrid.yaml defaults)
    python examples/circle_quick.py 192.168.1.8

    # tighter / faster / no Z force
    python examples/circle_quick.py 192.168.1.8 --profile circle \\
        --radius 0.03 --period 3 --duration 12

    # toggle a single field on top of a profile (sticky cache)
    python examples/circle_quick.py 192.168.1.8 --profile circle \\
        --linear-interp 0 --inner-v-filter-alpha 0.1

Prereq: daemon up with FT:
    ./fr3-stack up hybrid --ft
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from fr3_stack import Robot


SAFE_FORCE_THRESHOLDS  = [30.0, 30.0, 30.0, 8.0, 8.0, 8.0]
SAFE_TORQUE_THRESHOLDS = [15.0] * 7


def countdown(n: int, msg: str) -> None:
    for i in range(n, 0, -1):
        print(f"  {msg} in {i}s ...  (Ctrl+C to abort)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 70, end="\r")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost")
    p.add_argument("--profile",  default=None,
                   help="hybrid profile name (loads hybrid.<name>.yaml)")
    p.add_argument("--dt",       type=float, default=0.01)
    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--radius",   type=float, default=0.05)
    p.add_argument("--period",   type=float, default=4.0)
    p.add_argument("--force-n",  type=float, default=3.0,
                   help="Z press force magnitude in N (sent as -Z)")
    p.add_argument("--no-ft",    action="store_true")
    p.add_argument("--unsafe",   action="store_true",
                   help="skip 3-second countdown")

    # Per-call profile overrides — set sticky cache slots without editing yaml.
    p.add_argument("--linear-interp",        type=int, choices=(0, 1), default=None)
    p.add_argument("--inner-v-filter-alpha", type=float, default=None)
    args = p.parse_args()

    require_ft = not args.no_ft

    with Robot(args.host) as robot:
        if args.profile is not None:
            robot.set_profile("hybrid", args.profile)
            print(f"profile loaded: hybrid.{args.profile}.yaml")

        s = robot.wait_for_state()
        if require_ft and not s.has_ft_sensor:
            print("ERROR: no FT sensor — start daemon with --ft, or rerun --no-ft.")
            sys.exit(1)

        p0 = s.pos.copy()
        q0 = s.quat_xyzw.copy()
        print(f"pos0={p0.round(4)}  controller={s.controller}  "
              f"has_ft={s.has_ft_sensor}")
        print(f"r={args.radius*1000:.0f}mm  period={args.period}s  "
              f"duration={args.duration}s  dt={args.dt*1000:.0f}ms  "
              f"Fz=-{args.force_n}N  S=[1,1,0,1,1,1]")

        if not args.unsafe:
            countdown(3, "start")

        # Record actual vs commanded for radial-error stats.
        rows: list[list[float]] = []

        def target_fn(t):
            th = 2.0 * math.pi * t / args.period
            p_d = p0.copy()
            p_d[0] += args.radius * (math.cos(th) - 1.0)
            p_d[1] += args.radius * math.sin(th)
            cur = robot.state
            if cur.valid:
                rows.append([t, *p_d.tolist(), *cur.pos.tolist()])
            return p_d, q0

        kwargs = dict(
            duration          = args.duration,
            target_force      = [0.0, 0.0, -args.force_n, 0.0, 0.0, 0.0],
            S                 = [1, 1, 0, 1, 1, 1],
            target_fn         = target_fn,
            dt                = args.dt,
            force_thresholds  = SAFE_FORCE_THRESHOLDS,
            torque_thresholds = SAFE_TORQUE_THRESHOLDS,
            require_ft_sensor = require_ft,
        )
        if args.linear_interp is not None:
            kwargs["linear_interp"] = bool(args.linear_interp)
        if args.inner_v_filter_alpha is not None:
            kwargs["inner_v_filter_alpha"] = args.inner_v_filter_alpha

        try:
            robot.run_hybrid_force_position(**kwargs)
        finally:
            print("\nrelaxing to gravity-comp ...")
            robot.send_idle()

        if rows:
            a = np.asarray(rows)
            center = p0.copy(); center[0] -= args.radius
            r_d = np.linalg.norm(a[:, 1:3] - center[:2], axis=1)
            r_m = np.linalg.norm(a[:, 4:6] - center[:2], axis=1)
            err_mm = (r_m - r_d) * 1000
            print(f"N={len(a)}  radial err  rms={np.sqrt((err_mm**2).mean()):.3f} mm  "
                  f"peak={np.abs(err_mm).max():.3f} mm")


if __name__ == "__main__":
    main()
