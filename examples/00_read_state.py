"""Print FR3 state from the daemon. No commands sent — read-only.

The daemon's state stream goes through ZMQ PUB at 200 Hz; this client
subscribes and prints either a single snapshot or a streaming view.

Usage:
    python examples/00_read_state.py 192.168.1.8                # one snapshot
    python examples/00_read_state.py 192.168.1.8 --stream       # ~10 Hz pretty
    python examples/00_read_state.py 192.168.1.8 --stream --hz 2

The host is the NUC running fr3-stack, NOT the robot's IP.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from fr3_stack import Robot


def fmt(name: str, arr, n: int = 4) -> str:
    return f"{name:11s} {np.array2string(np.asarray(arr), precision=n, suppress_small=True)}"


def print_snapshot(s) -> None:
    print(f"controller   {s.controller}    running={s.running}    "
          f"valid={s.valid}    last_error='{s.last_error}'")
    print(fmt("q (rad)",     s.q))
    print(fmt("dq (rad/s)",  s.dq))
    print(fmt("pos (m)",     s.pos))
    print(fmt("quat xyzw",   s.quat_xyzw))
    # libfranka's O_F_ext_hat_K — derived from joint torques, NOT a real
    # sensor. ~3-5 N noise floor, biases with payload. Base frame.
    print(fmt("F_franka_est", s.wrench_ext, n=2))
    # Calibrated FT-sensor reading rotated to base via R_O_EE. None means no
    # --ft-sensor-kind passed to fr3-stack (or backend not yet ready).
    if s.has_ft_sensor:
        print(fmt("F_ft_meas",   s.wrench_ft, n=2))
    else:
        print("F_ft_meas   (none — no FT sensor backend attached)")
    print(f"timestamp    {s.timestamp:.3f} s")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("--stream", action="store_true",
                   help="keep printing instead of one-shot")
    p.add_argument("--hz", type=float, default=10.0,
                   help="streaming print rate (default 10)")
    args = p.parse_args()

    with Robot(args.host) as robot:
        s = robot.wait_for_state(timeout=5.0)

        if not args.stream:
            print_snapshot(s)
            return

        # Streaming: clear screen + repaint to keep output stable. Press
        # Ctrl+C to stop. The daemon publishes at ~200 Hz; we sample at --hz.
        period = 1.0 / args.hz
        try:
            while True:
                s = robot.state
                print("\033[2J\033[H", end="")   # clear + home
                print(f"fr3 state   host={args.host}   "
                      f"@ {time.strftime('%H:%M:%S')}   "
                      f"sample {args.hz:.0f} Hz")
                print("-" * 60)
                print_snapshot(s)
                time.sleep(period)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
