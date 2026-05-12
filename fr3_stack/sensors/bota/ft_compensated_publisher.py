"""Stream the gravity+bias-compensated FT wrench as CSV on stdout.

7 cols by default (``t,fx,fy,fz,tx,ty,tz``); ``--raw`` adds 6 columns for
the uncompensated stream. ``--frame sensor`` switches from base to sensor
frame.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from fr3_stack import Robot

from ._common import (
    DEFAULT_HOST,
    GRAVITY,
    default_calib_path,
    quat_xyzw_to_R,
)

DEFAULT_HZ      = 1000.0    # match daemon RT rate so plots line up tick-for-tick.
DEFAULT_FRAME   = "base"
DEFAULT_LOWPASS = 0.0       # first-order LP cutoff in Hz; 0 disables


def compensate(state, calib: dict, frame: str) -> tuple[np.ndarray, np.ndarray] | None:
    """``(f_comp, t_comp)`` or None if no FT data."""
    if not state.has_ft_sensor:
        return None

    mass   = calib["mass"]
    com_s  = np.asarray(calib["center_of_mass"])
    f_bias = np.asarray(calib["force_bias"])
    t_bias = np.asarray(calib["torque_bias"])
    mg     = mass * GRAVITY

    R       = quat_xyzw_to_R(state.quat_xyzw)
    w_base  = state.wrench_ft
    # Bias / CoM live in the sensor frame; back-transform from base first.
    f_raw_s = R.T @ w_base[0:3]
    t_raw_s = R.T @ w_base[3:6]

    f_grav_s = R.T @ np.array([0.0, 0.0, -mg])
    t_grav_s = np.cross(com_s, f_grav_s)

    f_comp_s = f_raw_s - f_grav_s - f_bias
    t_comp_s = t_raw_s - t_grav_s - t_bias

    if frame == "sensor":
        return f_comp_s, t_comp_s
    return R @ f_comp_s, R @ t_comp_s


def raw_wrench(state, frame: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Uncompensated wrench. Prefers ``state.wrench_ft_raw``; falls back to
    ``state.wrench_ft`` for older daemons with a single uncompensated stream."""
    if not state.has_ft_sensor:
        return None
    w_base = state.wrench_ft_raw if state.wrench_ft_raw is not None \
                                 else state.wrench_ft
    if frame == "sensor":
        R = quat_xyzw_to_R(state.quat_xyzw)
        return R.T @ w_base[0:3], R.T @ w_base[3:6]
    return w_base[0:3].copy(), w_base[3:6].copy()


def compensated_wrench(state, calib: dict | None, frame: str
                       ) -> tuple[np.ndarray, np.ndarray] | None:
    """``(f, t)`` compensated. Daemon-side ⇒ pass-through; daemon-raw +
    Python calib ⇒ :func:`compensate`; daemon-raw + no calib ⇒ None."""
    if not state.has_ft_sensor:
        return None
    if state.ft_compensated:
        w_base = state.wrench_ft
        if frame == "sensor":
            R = quat_xyzw_to_R(state.quat_xyzw)
            return R.T @ w_base[0:3], R.T @ w_base[3:6]
        return w_base[0:3].copy(), w_base[3:6].copy()
    if calib is None:
        return None
    return compensate(state, calib, frame)


class LowPass:
    """Per-sample-dt first-order RC; dt from monotonic timestamps."""
    def __init__(self, cutoff_hz: float):
        self.rc = 1.0 / (2.0 * np.pi * cutoff_hz) if cutoff_hz > 0 else 0.0
        self.x: np.ndarray | None = None
        self.t: float | None = None

    def step(self, x: np.ndarray, t: float) -> np.ndarray:
        if self.rc <= 0.0 or self.x is None or self.t is None:
            self.x, self.t = x.copy(), t
            return x
        dt = t - self.t
        if dt <= 0.0:
            return self.x
        alpha = dt / (self.rc + dt)
        self.x = alpha * x + (1.0 - alpha) * self.x
        self.t = t
        return self.x


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default=DEFAULT_HOST,
                   help=f"NUC host or IP running fr3d (default {DEFAULT_HOST})")
    p.add_argument("--hz", type=float, default=DEFAULT_HZ,
                   help=f"output rate (default {DEFAULT_HZ})")
    p.add_argument("--frame", choices=("base", "sensor"), default=DEFAULT_FRAME,
                   help=f"frame for the output wrench (default {DEFAULT_FRAME})")
    p.add_argument("--lowpass", type=float, default=DEFAULT_LOWPASS,
                   help=f"first-order LP cutoff in Hz (default {DEFAULT_LOWPASS}, 0 = off)")
    p.add_argument("--raw", action="store_true",
                   help="emit raw (uncompensated) wrench in 6 extra columns "
                        "(fx_raw,fy_raw,fz_raw,tx_raw,ty_raw,tz_raw) for "
                        "fr3-ft-plot's Comp/Raw/Both toggle")
    p.add_argument("--calib", type=Path, default=default_calib_path(),
                   help="path to calibration YAML (default %(default)s)")
    args = p.parse_args()

    # Calib only needed when daemon is uncompensated (state.ft_compensated=False).
    calib: dict | None = None
    if args.calib.exists():
        with open(args.calib) as fp:
            calib = yaml.safe_load(fp)
        print(f"# mass={calib['mass']:.4f}kg  com={calib['center_of_mass']}",
              file=sys.stderr)
        print(f"# f_bias={calib['force_bias']}  t_bias={calib['torque_bias']}",
              file=sys.stderr)
    else:
        print(f"# no calib at {args.calib} — relying on daemon-side compensation "
              "(state.ft_compensated must be True)", file=sys.stderr)
    print(f"# frame={args.frame}  hz={args.hz}  lowpass={args.lowpass}  "
          f"raw={args.raw}",
          file=sys.stderr)
    if args.raw:
        print("t,fx,fy,fz,tx,ty,tz,fx_raw,fy_raw,fz_raw,tx_raw,ty_raw,tz_raw")
    else:
        print("t,fx,fy,fz,tx,ty,tz")

    lp_f = LowPass(args.lowpass)
    lp_t = LowPass(args.lowpass)
    # Separate LP state per stream — else raw would track compensated through the LP.
    lp_f_raw = LowPass(args.lowpass) if args.raw else None
    lp_t_raw = LowPass(args.lowpass) if args.raw else None
    period = 1.0 / args.hz

    with Robot(args.host) as robot:
        s_first = robot.wait_for_state(timeout=5.0)
        if s_first.ft_compensated:
            print("# compensation: DAEMON (state.ft_compensated=True) — "
                  "Python pass-through", file=sys.stderr)
        elif calib is not None:
            print("# compensation: PYTHON (daemon publishing raw)", file=sys.stderr)
        else:
            print("ERROR: daemon is publishing raw wrench AND no calib found "
                  "at --calib; cannot produce compensated stream.\n"
                  "Either run `fr3-ft-calibrate` and restart the daemon, "
                  "or pass --calib pointing at an existing YAML.",
                  file=sys.stderr)
            sys.exit(1)

        try:
            while True:
                tick = time.monotonic()
                s = robot.state
                out = compensated_wrench(s, calib, args.frame)
                if out is not None:
                    f, t = out
                    f = lp_f.step(f, s.timestamp)
                    t = lp_t.step(t, s.timestamp)
                    row = (f"{s.timestamp:.6f},"
                           f"{f[0]:+.6f},{f[1]:+.6f},{f[2]:+.6f},"
                           f"{t[0]:+.6f},{t[1]:+.6f},{t[2]:+.6f}")
                    if args.raw:
                        raw = raw_wrench(s, args.frame)
                        if raw is not None:
                            fr, tr = raw
                            fr = lp_f_raw.step(fr, s.timestamp)
                            tr = lp_t_raw.step(tr, s.timestamp)
                            row += (f",{fr[0]:+.6f},{fr[1]:+.6f},{fr[2]:+.6f},"
                                    f"{tr[0]:+.6f},{tr[1]:+.6f},{tr[2]:+.6f}")
                        else:
                            # Keep column count consistent for fr3-ft-plot.
                            row += ",0,0,0,0,0,0"
                    print(row, flush=True)
                sleep_for = period - (time.monotonic() - tick)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
