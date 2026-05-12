"""A/B test: hybrid circle with vs without daemon-side LERP.

Streams the same 5 cm circle (Z held in -3 N force) twice — once with the
new ``linear_interp=True`` bridge engaged and once with it off. Logs the
commanded p_d and measured p at each tick into two CSVs and prints
radial-error stats so you can see the streaming-step-train chatter.

Run after rebuilding the daemon with the LERP changes:
    ./fr3-stack up hybrid -d --ft
    python examples/circle_lerp_compare.py <nuc-host> --dt 0.01

Args:
    --dt           streaming period [s] (try 0.01 = 100 Hz, then 0.005)
    --duration     seconds per leg
    --radius       circle radius [m]
    --period       seconds per revolution
    --no-ft        skip FT-sensor gate (debug/no sensor)
    --out          directory for the two CSVs (default ./)
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

from fr3_stack import Robot


SAFE_FORCE_THRESHOLDS  = [30.0, 30.0, 30.0, 8.0, 8.0, 8.0]
SAFE_TORQUE_THRESHOLDS = [15.0] * 7


def countdown(seconds: int, msg: str) -> None:
    for i in range(seconds, 0, -1):
        print(f"  {msg} in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 70, end="\r")


def run_leg(robot: Robot, args, *, linear_interp: bool, out_csv: Path) -> None:
    """One pass of the circle. Records t,p_d_x,p_d_y,p_d_z,p_x,p_y,p_z."""
    label = "LERP=ON" if linear_interp else "LERP=OFF"
    countdown(3, f"circle [{label}] r={args.radius*1000:.0f}mm "
                  f"period={args.period}s dt={args.dt*1000:.0f}ms")
    s = robot.wait_for_state()
    p0 = s.pos.copy()
    q0 = s.quat_xyzw.copy()
    rows: list[list[float]] = []

    def target_fn(t):
        theta = 2.0 * math.pi * t / args.period
        p = p0.copy()
        p[0] += args.radius * (math.cos(theta) - 1.0)
        p[1] += args.radius * math.sin(theta)
        return p, q0

    # First tick installs S / Tr / thresholds / linear_interp on the cache.
    pos0, quat0 = target_fn(0.0)
    robot.send_hybrid_force_position(
        target_pos=pos0,
        target_quat_xyzw=quat0,
        target_force=[0.0, 0.0, -3.0, 0.0, 0.0, 0.0],
        S=[1, 1, 0, 1, 1, 1],
        force_thresholds  = SAFE_FORCE_THRESHOLDS,
        torque_thresholds = SAFE_TORQUE_THRESHOLDS,
        linear_interp     = linear_interp,
        require_ft_sensor = not args.no_ft,
    )

    t_start = time.monotonic()
    next_t  = t_start
    while True:
        t = time.monotonic() - t_start
        if t >= args.duration:
            break
        p_d, _ = target_fn(t)
        robot.send_hybrid_force_position(
            target_pos=p_d,
            target_quat_xyzw=q0,
            target_force=[0.0, 0.0, -3.0, 0.0, 0.0, 0.0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )
        st = robot.state
        if st.valid:
            rows.append([t, *p_d.tolist(), *st.pos.tolist()])
        next_t += args.dt
        sleep_for = next_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.monotonic()

    a = np.asarray(rows)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out_csv, a, delimiter=",", fmt="%.6f",
               header="t,p_d_x,p_d_y,p_d_z,p_x,p_y,p_z", comments="")

    # Radial error in the XY plane around the circle center (p0 - [r,0,0]).
    center = p0.copy()
    center[0] -= args.radius
    r_d  = np.linalg.norm(a[:, 1:3] - center[:2], axis=1)
    r_m  = np.linalg.norm(a[:, 4:6] - center[:2], axis=1)
    err  = (r_m - r_d) * 1000  # mm
    print(f"[{label}]  N={len(a)}  "
          f"radial err  rms={np.sqrt((err**2).mean()):.3f} mm  "
          f"peak={np.abs(err).max():.3f} mm  →  {out_csv}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost")
    p.add_argument("--dt",       type=float, default=0.01)
    p.add_argument("--duration", type=float, default=8.0)
    p.add_argument("--radius",   type=float, default=0.05)
    p.add_argument("--period",   type=float, default=4.0)
    p.add_argument("--no-ft",    action="store_true")
    p.add_argument("--out",      type=Path,  default=Path("."))
    args = p.parse_args()

    with Robot(args.host) as robot:
        s = robot.wait_for_state()
        print(f"connected.  pos0={np.round(s.pos, 4)}  "
              f"controller={s.controller}  has_ft={s.has_ft_sensor}")

        try:
            run_leg(robot, args, linear_interp=True,
                    out_csv=args.out / "circle_lerp_on.csv")
            print("relaxing 2s before the off-LERP leg ...")
            robot.send_idle()
            time.sleep(2.0)

            run_leg(robot, args, linear_interp=False,
                    out_csv=args.out / "circle_lerp_off.csv")
        finally:
            print("relaxing to gravity-comp ...")
            robot.send_idle()


if __name__ == "__main__":
    main()
