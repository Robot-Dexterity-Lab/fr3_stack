"""Smoke test for the min-jerk move-to-pose generator (Command.moveTo).

What it does:
    1. reads the current pose
    2. picks a goal = current + (dx, dy, dz)   (default small +Z)
    3. calls send_move_to(goal, run_time=T) and polls state at `--rate` Hz
    4. logs (t, pos) during the move, then verifies:
         * final position converged to within `--tol` of goal
         * empirical peak speed ≈ analytic 1.875·|Δp|/T (min-jerk hallmark)

Safety:
    * per-axis delta clamped to ±10 cm (override with --unsafe)
    * 3-second countdown before the arm moves (override with --unsafe)
    * low default impedance gains so a wrong target stays gentle
    * idle on exit (gravity comp) — robot is *not* held at goal after exit

Usage:
    python examples/02_move_to.py <nuc-host> [dx dy dz] [-T 1.5]

Example:
    python examples/02_move_to.py 192.168.1.8 0 0 0.05 -T 1.5
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
    print(" " * 60, end="\r")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("delta", nargs="*", type=float, default=[0.0, 0.0, 0.05],
                   help="dx dy dz in meters (default: 0 0 0.05)")
    p.add_argument("-T", "--run-time", type=float, default=None,
                   help="trajectory duration [s]; default = max(1.0, 4·|Δp|)")
    p.add_argument("--rate", type=float, default=200.0,
                   help="state polling rate Hz (default 200)")
    p.add_argument("--tol", type=float, default=2e-3,
                   help="convergence tolerance [m] for the final pose check")
    p.add_argument("--K-xy",  type=float, default=SAFE_K_XY)
    p.add_argument("--K-z",   type=float, default=SAFE_K_Z)
    p.add_argument("--K-rot", type=float, default=SAFE_K_ROT)
    p.add_argument("--return-home", action="store_true",
                   help="after reaching goal, run a second moveTo back to start")
    p.add_argument("--unsafe", action="store_true",
                   help="skip delta clamp and confirmation countdown")
    args = p.parse_args()

    if len(args.delta) != 3:
        p.error("delta needs exactly 3 numbers (dx dy dz)")
    dx, dy, dz = args.delta

    if not args.unsafe:
        clipped = [max(-SAFE_MAX_DELTA, min(SAFE_MAX_DELTA, d))
                   for d in (dx, dy, dz)]
        if clipped != [dx, dy, dz]:
            print(f"⚠️  delta clamped to ±{SAFE_MAX_DELTA*100:.0f} cm: "
                  f"{(dx, dy, dz)} → {tuple(clipped)} (--unsafe to bypass)")
            dx, dy, dz = clipped

    delta = np.array([dx, dy, dz])
    dist  = float(np.linalg.norm(delta))
    # Default run_time: 1 s minimum, scale up at ~25 cm/s peak speed.
    T = args.run_time if args.run_time is not None else max(1.0, 4.0 * dist)

    K = np.array([args.K_xy, args.K_xy, args.K_z,
                  args.K_rot, args.K_rot, args.K_rot])

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state()
        print(f"connected.  pos0 = {np.round(s0.pos, 4)}  "
              f"controller={s0.controller}")

        start_pos = s0.pos.copy()
        goal_pos  = start_pos + delta

        # Analytic min-jerk peak speed for 1-D move of length L over time T:
        #   v_peak = 1.875 · L / T   (occurs at t = T/2)
        v_peak_analytic = 1.875 * dist / T
        a_peak_analytic = 5.7735 * dist / (T * T)

        print(f"plan:")
        print(f"  from   {np.round(start_pos, 4)}")
        print(f"  to     {np.round(goal_pos, 4)}  "
              f"(Δ {dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m  |Δp|={dist:.3f} m")
        print(f"  T      {T:.2f} s  →  v_peak≈{v_peak_analytic*100:.1f} cm/s, "
              f"a_peak≈{a_peak_analytic*100:.1f} cm/s²")
        print(f"  K      xy={args.K_xy:.0f}  z={args.K_z:.0f}  "
              f"rot={args.K_rot:.0f}")
        print(f"  poll   {args.rate:.0f} Hz, tol {args.tol*1000:.1f} mm")

        if not args.unsafe:
            try:
                countdown(3, "starting moveTo")
            except KeyboardInterrupt:
                print("\nabort. arm untouched.")
                return

        # --- Fire the moveTo command (single-shot — daemon takes it from here)
        robot.send_move_to(
            target_pos       = goal_pos,
            target_quat_xyzw = s0.quat_xyzw,   # keep start orientation
            run_time         = T,
            K                = K,
        )
        t0 = time.monotonic()

        # --- Telemetry: poll state until run_time + slack, log positions ---
        log_t   = []
        log_pos = []
        dt = 1.0 / args.rate
        deadline = t0 + T + 0.5     # half-second slack for hold check

        try:
            while True:
                now = time.monotonic()
                if now > deadline:
                    break
                s = robot.state
                log_t.append(now - t0)
                log_pos.append(s.pos.copy())
                # Periodic print, ~5 Hz
                if int((now - t0) * 5) != int((now - t0 - dt) * 5):
                    err = s.pos - goal_pos
                    print(f"  t={now-t0:5.2f}s  pos={np.round(s.pos, 4)}  "
                          f"err={np.round(err*1000, 1)} mm  "
                          f"ctrl={s.controller}")
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\ninterrupted — sending idle ...")
            robot.send_idle()
            return

        log_t   = np.array(log_t)
        log_pos = np.array(log_pos)

        # --- Convergence check ----------------------------------------------
        final_err = log_pos[-1] - goal_pos
        err_norm  = float(np.linalg.norm(final_err))
        ok_conv   = err_norm < args.tol
        print(f"\nfinal err = {np.round(final_err*1000, 2)} mm  "
              f"(|err|={err_norm*1000:.2f} mm, tol {args.tol*1000:.1f} mm)  "
              f"→ {'OK' if ok_conv else 'FAIL'}")

        # --- Empirical peak speed vs analytic --------------------------------
        # Numerical derivative on the polled stream. We only look at samples
        # inside [0, T] — past T the trajectory is finished and any motion is
        # just impedance settling.
        mask = log_t <= T
        t_m  = log_t[mask]
        p_m  = log_pos[mask]
        if len(t_m) >= 4:
            dp = np.diff(p_m, axis=0)
            dt_m = np.diff(t_m)
            v = np.linalg.norm(dp, axis=1) / np.maximum(dt_m, 1e-6)
            v_peak_emp = float(v.max())
            t_at_peak  = float(0.5 * (t_m[v.argmax()] + t_m[v.argmax() + 1]))
            ratio = v_peak_emp / max(v_peak_analytic, 1e-9)
            # Min-jerk peaks at t=T/2; allow ±25 % on speed (impedance lag,
            # polling jitter, finite differencing) and ±15 % on timing.
            ok_peak = (0.6 < ratio < 1.4) and (abs(t_at_peak - T/2) < 0.15 * T)
            print(f"v_peak empirical = {v_peak_emp*100:.1f} cm/s @ t={t_at_peak:.2f}s")
            print(f"v_peak analytic  = {v_peak_analytic*100:.1f} cm/s @ t={T/2:.2f}s "
                  f"→ ratio {ratio:.2f}  → {'OK' if ok_peak else 'CHECK'}")
        else:
            ok_peak = False
            print("not enough samples to estimate v_peak (rate too low)")

        # --- Optional return-home leg ---------------------------------------
        if args.return_home and ok_conv:
            print("\nreturning home with a second moveTo ...")
            robot.send_move_to(
                target_pos       = start_pos,
                target_quat_xyzw = s0.quat_xyzw,
                run_time         = T,
                K                = K,
            )
            time.sleep(T + 0.3)
            s = robot.state
            print(f"  back at {np.round(s.pos, 4)}  "
                  f"err={np.round((s.pos - start_pos)*1000, 1)} mm")

        # Hand the arm back to gravity-comp before disconnecting.
        robot.send_idle()
        time.sleep(0.1)
        print("idled.")

        if not (ok_conv and ok_peak):
            sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
