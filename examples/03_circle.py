"""Trace a 5 cm circle in the XY plane while logging external wrench.

Uses the high-level :class:`Arm` API: stiffness is set once up front, then
the loop just streams :class:`Pose` targets via ``arm.send``.

Usage:
    uv run python examples/03_circle.py <nuc-host>
"""
from __future__ import annotations

import sys
import time

import numpy as np

from fr3_stack import Arm, Pose


RADIUS      = 0.05     # m
FREQ_HZ     = 0.5      # one revolution every 2 s
DURATION    = 10.0     # s
SEND_RATE   = 100.0    # Hz — outer send rate to daemon
TARGET_RATE = 10.0     # Hz — new trajectory target every 0.1 s
PRINT_DT    = 0.5      # s between status prints


def main(host: str) -> None:
    with Arm(host) as arm:
        anchor = arm.observe().pose
        print(f"connected. initial pos = {np.round(anchor.pos, 3)}")

        # Stiff Z, soft XY, moderate rotation. damp_ratio=0.9 → D = 2·√K·0.9.
        arm.set_stiffness(K=[100, 100, 800, 30, 30, 30], damp_ratio=0.9)

        send_period   = 1.0 / SEND_RATE
        target_period = 1.0 / TARGET_RATE
        t0 = time.monotonic()
        next_t        = t0
        next_target_t = 0.0
        next_print    = 0.0
        target = Pose(anchor.pos.copy(), anchor.quat.copy())

        while (t := time.monotonic() - t0) < DURATION:
            # Recompute trajectory target at TARGET_RATE; in between we
            # re-send the same Pose, so the daemon sees SEND_RATE cmds with
            # SEND_RATE / TARGET_RATE duplicates per unique target.
            if t >= next_target_t:
                ang = 2 * np.pi * FREQ_HZ * t
                target = Pose(
                    anchor.pos + np.array([
                        RADIUS * (np.cos(ang) - 1.0),
                        RADIUS *  np.sin(ang),
                        0.0,
                    ]),
                    anchor.quat.copy(),
                )
                next_target_t += target_period

            arm.send(target)

            if t >= next_print:
                obs = arm.observe()
                err = obs.pose.pos - target.pos
                print(f"t={t:5.2f}  pos_err={np.round(err, 4)}  "
                      f"F_ext={np.round(obs.wrench[:3], 2)}")
                next_print += PRINT_DT

            next_t += send_period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

        # Hand the arm back to gravity-only mode before quitting.
        arm.relax()


if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    main(host)
