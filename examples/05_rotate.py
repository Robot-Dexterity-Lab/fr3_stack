"""Rotate the EE about each body axis while logging angular tracking error.

Rotation analogue of ``03_circle.py``. Holds position fixed at the anchor
and sinusoidally swings the EE orientation by ±``AMP_DEG`` around body Z,
then Y, then X. Logs angular error and external torque each ~0.5 s.

Uses ``linear_interp=True, ema=False`` — the recommended combo for 10 Hz
target updates: LERP alone gives a continuous 1 kHz target without the
~19 ms phase lag the EMA would add.

Usage:
    uv run python examples/05_rotate.py <nuc-host>
"""
from __future__ import annotations

import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from fr3_stack import Arm, Pose


AMP_DEG     = 15.0    # peak swing amplitude per axis [deg]
FREQ_HZ     = 0.25    # one full cycle per 4 s
DURATION    = 8.0     # s per axis (≈ 2 full cycles)
SEND_RATE   = 100.0   # Hz, outer command loop
TARGET_RATE = 10.0    # Hz, new trajectory point every 100 ms
PRINT_DT    = 0.5     # s between status prints

AXES = [
    ("Z (yaw)",   np.array([0.0, 0.0, 1.0])),
    ("Y (pitch)", np.array([0.0, 1.0, 0.0])),
    ("X (roll)",  np.array([1.0, 0.0, 0.0])),
]


def angle_err_deg(q_act_xyzw, q_tgt_xyzw) -> float:
    """Magnitude of (q_act ⁻¹ · q_tgt) as an angle in degrees."""
    r_act = R.from_quat(q_act_xyzw)
    r_tgt = R.from_quat(q_tgt_xyzw)
    return float(np.linalg.norm((r_act.inv() * r_tgt).as_rotvec()) * 180.0 / np.pi)


def angle_from_anchor_deg(q_act_xyzw, q_anchor_xyzw) -> float:
    """How far q_act has rotated away from q_anchor, in degrees."""
    r_act    = R.from_quat(q_act_xyzw)
    r_anchor = R.from_quat(q_anchor_xyzw)
    return float(np.linalg.norm((r_anchor.inv() * r_act).as_rotvec()) * 180.0 / np.pi)


def sweep_axis(arm: Arm, anchor: Pose, axis_name: str, axis_unit: np.ndarray) -> None:
    print(f"\n--- body {axis_name}: ±{AMP_DEG:.0f}° @ {FREQ_HZ} Hz, {DURATION:.0f} s ---")
    print(f"  ctrl mode reported by daemon: {arm.robot.state.controller!r}")

    amp_rad  = np.deg2rad(AMP_DEG)
    R_anchor = R.from_quat(anchor.quat)

    send_period   = 1.0 / SEND_RATE
    target_period = 1.0 / TARGET_RATE
    t0            = time.monotonic()
    next_t        = t0
    next_target_t = 0.0
    next_print    = 0.0
    target        = Pose(anchor.pos.copy(), anchor.quat.copy())

    while (t := time.monotonic() - t0) < DURATION:
        if t >= next_target_t:
            theta    = amp_rad * np.sin(2 * np.pi * FREQ_HZ * t)
            r_target = R_anchor * R.from_rotvec(theta * axis_unit)
            target   = Pose(anchor.pos.copy(), r_target.as_quat())
            next_target_t += target_period

        arm.send(target)

        if t >= next_print:
            obs = arm.observe()
            err_deg     = angle_err_deg(obs.pose.quat, target.quat)
            actual_deg  = angle_from_anchor_deg(obs.pose.quat, anchor.quat)
            target_deg  = angle_from_anchor_deg(target.quat,    anchor.quat)
            tau_ext = np.round(obs.wrench[3:6], 2)
            print(f"  t={t:5.2f}  tgt={target_deg:+6.2f}°  act={actual_deg:+6.2f}°  "
                  f"err={err_deg:5.2f}°  τ_ext={tau_ext}")
            next_print += PRINT_DT

        next_t += send_period
        sleep_for = next_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.monotonic()


def main(host: str) -> None:
    with Arm(host) as arm:
        anchor = arm.observe().pose
        print(f"connected. initial pos  = {np.round(anchor.pos, 3)}")
        print(f"initial quat (xyzw)    = {np.round(anchor.quat, 3)}")

        # Translation K kept high so the anchor doesn't drift while rotating.
        # Rotation K beefier than the [20,20,20] default — at K_rot=20 the EE
        # just follows your hand, but we want it to actually track the sine.
        arm.set_stiffness(K=[400, 400, 400, 60, 60, 60], damp_ratio=0.9)
        arm.set_smoothing(linear_interp=True, ema=False)

        for axis_name, axis_unit in AXES:
            sweep_axis(arm, anchor, axis_name, axis_unit)
            # Return to anchor before the next axis so each sweep starts from
            # the same orientation.
            arm.move_to(anchor, duration=1.5)

        arm.relax()
        print("\ndone. arm is in gravity-comp.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <nuc-host>", file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1])
