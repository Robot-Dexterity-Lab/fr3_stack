"""Long-range Cartesian reset using Ruckig as the motion generator.

Streams jerk-limited targets into ``send_cartesian_impedance`` at high rate
instead of using the daemon's built-in min-jerk ``send_move_to``. Ruckig
respects per-axis velocity / acceleration / jerk caps, which gives you
explicit time-optimal motion under physical limits — useful for long
travels where the min-jerk shape (which scales peak speed with 1/T from a
single duration) is hard to tune.

Plan: Ruckig drives the (x, y, z) position with caller-set v/a/j caps.
Orientation is SLERPed from the start quaternion to the goal quaternion
over the same total duration Ruckig returns. Each control tick we send
``send_cartesian_impedance(pos, quat)`` with ``filter_alpha=1.0`` so the
daemon's EMA doesn't smear the already-smooth Ruckig output (same trick
the C++ ``MoveToCmd`` uses internally).

Usage:
    python examples/reset.py <nuc-host> --to X Y Z [--quat x y z w]
    python examples/reset.py <nuc-host> --rel dx dy dz
    python examples/reset.py <nuc-host> --home          # canonical fr3 home

Notes:
    * Requires ``ruckig`` in the active env: ``uv pip install ruckig`` or
      ``pip install ruckig``.
    * Quaternion convention is xyzw, matching the wire.
    * Defaults are safe long-travel limits: v_max=0.25 m/s, a_max=1.0 m/s²,
      j_max=5.0 m/s³, K=[150,150,300, 15,15,15]. Override per axis if you
      need faster motion and you've cleared the workspace.
    * On exit the arm is left in idle (gravity comp). Pass ``--hold`` to
      keep impedance live at the goal for N seconds before idling.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from fr3_stack import Robot

try:
    from ruckig import InputParameter, OutputParameter, Result, Ruckig
except ImportError as exc:
    raise SystemExit(
        "ruckig not installed in this env. Run:\n"
        "    uv pip install ruckig\n"
        "or  pip install ruckig"
    ) from exc


# Long-travel-safe defaults. These are conservative for an FR3 with the
# usual end-effectors; tighten or loosen per-task.
DEFAULT_V_MAX = 0.25      # m/s   per Cartesian axis
DEFAULT_A_MAX = 1.0       # m/s²
DEFAULT_J_MAX = 5.0       # m/s³
DEFAULT_K     = (150.0, 150.0, 300.0, 15.0, 15.0, 15.0)  # xy z rxyz
DEFAULT_RATE  = 200.0     # Hz, command stream rate

# Canonical FR3 "home" pose in base frame — matches the libfranka neutral
# joint config (q ≈ [0, -π/4, 0, -3π/4, 0, π/2, π/4]) forward-kinematic'd
# to the EE. If your tool offset differs, override with --to.
HOME_POS  = np.array([0.30702, 0.0, 0.4868])
HOME_QUAT = np.array([1.0, 0.0, 0.0, 0.0])   # xyzw, EE pointing -z


def countdown(seconds: int, msg: str) -> None:
    for i in range(seconds, 0, -1):
        print(f"  {msg} in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 70, end="\r")


def plan_and_run(
    robot:   Robot,
    pos0:    np.ndarray,
    quat0:   np.ndarray,
    pos_g:   np.ndarray,
    quat_g:  np.ndarray,
    *,
    v_max:   np.ndarray,
    a_max:   np.ndarray,
    j_max:   np.ndarray,
    K:       np.ndarray,
    rate:    float,
) -> tuple[float, np.ndarray]:
    """Stream a Ruckig-paced trajectory from pos0/quat0 to pos_g/quat_g.

    Returns (elapsed_seconds, final_state_pos).
    """
    dt = 1.0 / rate

    # --- Set up Ruckig over 3 DoF position ------------------------------------
    ruckig = Ruckig(3, dt)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.current_position     = pos0.tolist()
    inp.current_velocity     = [0.0, 0.0, 0.0]
    inp.current_acceleration = [0.0, 0.0, 0.0]
    inp.target_position      = pos_g.tolist()
    inp.target_velocity      = [0.0, 0.0, 0.0]
    inp.target_acceleration  = [0.0, 0.0, 0.0]
    inp.max_velocity         = v_max.tolist()
    inp.max_acceleration     = a_max.tolist()
    inp.max_jerk             = j_max.tolist()

    # First pass to get the synchronized duration so we can SLERP orientation.
    # Calling update() once consumes one tick; that's fine — we just remember
    # the total duration up front, then keep stepping inside the streaming loop.
    res = ruckig.update(inp, out)
    if res not in (Result.Working, Result.Finished):
        raise RuntimeError(f"ruckig.update failed: {res}")
    T = float(out.trajectory.duration)

    print(f"  ruckig duration: {T:.2f} s "
          f"(|Δp|={np.linalg.norm(pos_g - pos0):.3f} m, "
          f"v_max={v_max[0]:.2f} m/s, a_max={a_max[0]:.2f} m/s², "
          f"j_max={j_max[0]:.1f} m/s³)")

    # --- Orientation SLERP over [0, T] ---------------------------------------
    # Flip target sign if its inner product with start is negative — keeps
    # SLERP on the short arc. (Same convention as PoseTrajectoryInterpolator.)
    q0 = quat0 / np.linalg.norm(quat0)
    qg = quat_g / np.linalg.norm(quat_g)
    if np.dot(qg, q0) < 0:
        qg = -qg
    if T > 1e-6:
        slerp = Slerp([0.0, T], Rotation.from_quat(np.stack([q0, qg])))
    else:
        slerp = None  # zero-duration plan; just hold at goal

    # --- Stream targets at `rate` Hz -----------------------------------------
    t_start = time.monotonic()
    next_tick = t_start
    # Apply the first Ruckig step (which we already computed above).
    pos_cmd = np.asarray(out.new_position)
    quat_cmd = q0.copy() if slerp is None else slerp(0.0).as_quat()
    robot.send_cartesian_impedance(
        target_pos       = pos_cmd,
        target_quat_xyzw = quat_cmd,
        K                = K,
        filter_alpha     = 1.0,    # Ruckig output is C² smooth → no daemon EMA
        linear_interp    = False,  # we're dense, no need for daemon LERP
        ema              = False,
    )

    last_print = -1.0
    while res == Result.Working:
        # Step Ruckig forward by feeding back its previous output.
        out.pass_to_input(inp)
        res = ruckig.update(inp, out)
        if res not in (Result.Working, Result.Finished):
            raise RuntimeError(f"ruckig.update failed mid-stream: {res}")
        pos_cmd = np.asarray(out.new_position)

        t_now = time.monotonic() - t_start
        if slerp is not None:
            tc = min(max(t_now, 0.0), T)
            quat_cmd = slerp(tc).as_quat()

        robot.send_cartesian_impedance(
            target_pos       = pos_cmd,
            target_quat_xyzw = quat_cmd,
            K                = K,
            filter_alpha     = 1.0,
            linear_interp    = False,
            ema              = False,
        )

        if t_now - last_print > 0.25:
            s = robot.state
            err = s.pos - pos_g
            print(f"  t={t_now:5.2f}s  cmd={np.round(pos_cmd, 4)}  "
                  f"pos={np.round(s.pos, 4)}  "
                  f"err={np.round(err * 1000, 1)} mm  "
                  f"ctrl={s.controller}")
            last_print = t_now

        next_tick += dt
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            # Behind schedule — reset the anchor so we don't burst-send.
            next_tick = time.monotonic()

    # Ruckig is done — settle on the goal target for a few ticks so the
    # controller has time to converge to the final pose under impedance.
    settle_end = time.monotonic() + 0.3
    while time.monotonic() < settle_end:
        robot.send_cartesian_impedance(
            target_pos       = pos_g,
            target_quat_xyzw = qg,
            K                = K,
            filter_alpha     = 1.0,
            linear_interp    = False,
            ema              = False,
        )
        time.sleep(dt)

    elapsed = time.monotonic() - t_start
    final_pos = robot.state.pos.copy()
    return elapsed, final_pos


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")

    target = p.add_mutually_exclusive_group(required=True)
    target.add_argument("--to", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="absolute target position in base frame [m]")
    target.add_argument("--rel", nargs=3, type=float, metavar=("DX", "DY", "DZ"),
                        help="relative offset from current pose [m]")
    target.add_argument("--home", action="store_true",
                        help=f"go to canonical home pose {HOME_POS.tolist()}")

    p.add_argument("--quat", nargs=4, type=float, metavar=("X", "Y", "Z", "W"),
                   help="target quaternion xyzw (default: keep current orientation,"
                        " or HOME_QUAT with --home)")
    p.add_argument("--v-max", nargs="+", type=float, default=[DEFAULT_V_MAX],
                   help="max velocity [m/s], scalar or per-axis (default %(default)s)")
    p.add_argument("--a-max", nargs="+", type=float, default=[DEFAULT_A_MAX],
                   help="max acceleration [m/s²], scalar or per-axis")
    p.add_argument("--j-max", nargs="+", type=float, default=[DEFAULT_J_MAX],
                   help="max jerk [m/s³], scalar or per-axis")
    p.add_argument("--K", nargs=6, type=float, default=list(DEFAULT_K),
                   metavar=("Kx", "Ky", "Kz", "Krx", "Kry", "Krz"),
                   help="Cartesian impedance stiffness (default %(default)s)")
    p.add_argument("--rate", type=float, default=DEFAULT_RATE,
                   help="streaming rate [Hz] (default %(default)s)")
    p.add_argument("--hold", type=float, default=0.0,
                   help="hold impedance at goal for this many seconds before idling")
    p.add_argument("--tol", type=float, default=3e-3,
                   help="convergence tolerance [m] for the final pose check")
    p.add_argument("--unsafe", action="store_true",
                   help="skip the 3-second countdown")
    args = p.parse_args()

    def expand_caps(name: str, vals: list[float]) -> np.ndarray:
        if len(vals) == 1:
            return np.full(3, vals[0], dtype=float)
        if len(vals) == 3:
            return np.asarray(vals, dtype=float)
        p.error(f"--{name} expects 1 or 3 values, got {len(vals)}")

    v_max = expand_caps("v-max", args.v_max)
    a_max = expand_caps("a-max", args.a_max)
    j_max = expand_caps("j-max", args.j_max)
    K = np.asarray(args.K, dtype=float)

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state()
        print(f"connected.  pos0 = {np.round(s0.pos, 4)}  "
              f"quat0 = {np.round(s0.quat_xyzw, 3)}  ctrl={s0.controller}")

        pos0  = s0.pos.copy()
        quat0 = s0.quat_xyzw.copy()

        if args.home:
            pos_g  = HOME_POS.copy()
            quat_g = np.asarray(args.quat, dtype=float) if args.quat else HOME_QUAT.copy()
        elif args.to is not None:
            pos_g  = np.asarray(args.to, dtype=float)
            quat_g = np.asarray(args.quat, dtype=float) if args.quat else quat0
        else:  # --rel
            pos_g  = pos0 + np.asarray(args.rel, dtype=float)
            quat_g = np.asarray(args.quat, dtype=float) if args.quat else quat0

        dist = float(np.linalg.norm(pos_g - pos0))
        print("plan:")
        print(f"  from   pos={np.round(pos0, 4)}  quat={np.round(quat0, 3)}")
        print(f"  to     pos={np.round(pos_g, 4)}  quat={np.round(quat_g, 3)}  "
              f"|Δp|={dist:.3f} m")
        print(f"  caps   v_max={v_max.tolist()}  a_max={a_max.tolist()}  "
              f"j_max={j_max.tolist()}")
        print(f"  K      {K.tolist()}   rate={args.rate:.0f} Hz")

        if not args.unsafe:
            try:
                countdown(3, "starting ruckig reset")
            except KeyboardInterrupt:
                print("\nabort. arm untouched.")
                return

        try:
            elapsed, final_pos = plan_and_run(
                robot, pos0, quat0, pos_g, quat_g,
                v_max=v_max, a_max=a_max, j_max=j_max,
                K=K, rate=args.rate,
            )
        except KeyboardInterrupt:
            print("\ninterrupted — sending idle ...")
            robot.send_idle()
            time.sleep(0.1)
            return

        err = final_pos - pos_g
        err_norm = float(np.linalg.norm(err))
        ok = err_norm < args.tol
        print(f"\ndone in {elapsed:.2f}s.  final err = {np.round(err * 1000, 2)} mm "
              f"(|err|={err_norm * 1000:.2f} mm, tol {args.tol * 1000:.1f} mm)  "
              f"→ {'OK' if ok else 'CHECK'}")

        if args.hold > 0.0:
            print(f"holding at goal for {args.hold:.1f}s ...")
            t_end = time.monotonic() + args.hold
            dt = 1.0 / args.rate
            try:
                while time.monotonic() < t_end:
                    robot.send_cartesian_impedance(
                        target_pos       = pos_g,
                        target_quat_xyzw = quat_g,
                        K                = K,
                        filter_alpha     = 1.0,
                    )
                    time.sleep(dt)
            except KeyboardInterrupt:
                pass

        robot.send_idle()
        time.sleep(0.1)
        print("idled.")

        if not ok:
            sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
