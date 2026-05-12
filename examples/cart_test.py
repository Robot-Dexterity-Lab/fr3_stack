"""Real-machine experiment harness for cartesian_impedance via the daemon.

Mirror of src/bin/cartesian_test.cpp — same modes, same flags. Built on the
``Arm.record()`` / ``oscillate / step / disturb`` helpers, so this file is
the reference for "what running an experiment looks like in Python".

Modes (--mode):
  hold     : target = startup pose; for stability + disturbance-rejection
  osc      : sinusoid on --axis (--amp, --freq)
  step     : ``--settle`` seconds then a position step (--step) on --axis
  disturb  : same as hold; mode label exists so CSVs are easy to grep

Stiffness override: --k kx,ky,kz,krx,kry,krz (N/m, Nm/rad). Damping derived
from --damp-ratio.

Prereq: NUC daemon running (``./fr3-stack <config>``). For FT logging start
it with ``--ft``; otherwise the FT columns are NaN.

Usage:
    python examples/cart_test.py <nuc-host> [options]
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

from fr3_stack import Arm, Pose

_AXIS = {"x": 0, "y": 1, "z": 2}


def run_circle(arm: Arm, rec, *, plane: str, radius: float,
               period: float, duration: float) -> None:
    """Inline circle tracer — kept here (not in Arm) so the library stays
    focused on primitives. Uses the same target-loop pattern as
    Arm.oscillate / Arm.step.
    """
    ax_a, ax_b = _AXIS[plane[0]], _AXIS[plane[1]]
    omega = 2.0 * math.pi / period
    rec.mode = "circle"
    rec.axis = ax_a

    obs0 = arm.observe()
    p0   = obs0.pose.pos.copy()
    quat = obs0.pose.quat.copy()

    rate     = rec.rate_hz
    dt       = 1.0 / rate
    t_start  = time.monotonic()
    next_t   = t_start
    while True:
        now = time.monotonic()
        t = now - t_start
        if t >= duration:
            break
        p_d = p0.copy()
        # anchor at angle 0: trace starts at the anchor, orbits around
        # (anchor − r·â).
        p_d[ax_a] += radius * (math.cos(omega * t) - 1.0)
        p_d[ax_b] += radius * math.sin(omega * t)
        arm.send(Pose(p_d, quat))
        rec.push(t, p_d, arm.observe())
        next_t += dt
        sleep_for = next_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.monotonic()


def parse_K(s: str) -> np.ndarray:
    parts = s.split(",")
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            f"--k needs 6 comma-separated numbers, got {len(parts)}")
    return np.asarray([float(p) for p in parts])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("host", nargs="?", default="localhost",
                    help="NUC host or IP (default: localhost)")
    ap.add_argument("--mode", choices=["hold", "osc", "step", "disturb", "circle"],
                    default="hold")
    ap.add_argument("--axis", choices=["x", "y", "z"], default="z")
    ap.add_argument("--amp",  type=float, default=0.05)
    ap.add_argument("--freq", type=float, default=0.25)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--settle", type=float, default=2.0,
                    help="step mode: settle time before the jump (s)")
    ap.add_argument("--plane", choices=["xy", "xz", "yz"], default="xy",
                    help="circle mode: which base-frame plane")
    ap.add_argument("--radius", type=float, default=0.10,
                    help="circle mode: radius in m (default 10 cm)")
    ap.add_argument("--period", type=float, default=6.0,
                    help="circle mode: one revolution time in s")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--k", type=parse_K, default=None,
                    help="override stiffness (N/m, Nm/rad)")
    ap.add_argument("--damp-ratio", type=float, default=0.9)
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--png", type=Path, default=None,
                    help="save the auto-figure (matplotlib) to this path")
    args = ap.parse_args()

    with Arm(args.host) as arm:
        if args.k is not None:
            arm.set_stiffness(K=args.k, damp_ratio=args.damp_ratio)
        # Daemon uses J^T impedance (no OSC) — see
        # docs/controllers.md "Why we use J^T, not real OSC".

        with arm.record() as rec:
            if args.mode == "hold":
                arm.disturb(duration=args.duration)   # same wire, just 'hold' tag
                rec.mode = "hold"
            elif args.mode == "disturb":
                arm.disturb(duration=args.duration)
            elif args.mode == "osc":
                arm.oscillate(axis=args.axis, amp=args.amp, freq=args.freq,
                              duration=args.duration)
            elif args.mode == "step":
                arm.step(axis=args.axis, delta=args.step,
                         duration=args.duration, settle=args.settle)
            elif args.mode == "circle":
                run_circle(arm, rec,
                           plane=args.plane, radius=args.radius,
                           period=args.period, duration=args.duration)

        arm.relax()

    print(f"recorded {len(rec)} samples")
    for k, v in rec.metrics().items():
        print(f"  {k:20s} = {v}")

    if args.csv:
        rec.save(args.csv)
        print(f"wrote {args.csv}")
    if args.png:
        rec.plot(args.png)
        print(f"wrote {args.png}")


if __name__ == "__main__":
    main()
