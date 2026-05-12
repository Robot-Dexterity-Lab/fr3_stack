"""Manual FT-sensor payload calibration.

At each of ≥6 static poses, average a window of FT samples + EE rotation,
then LSQ-solve for ``{mass, CoM, f_bias, t_bias}`` satisfying::

    f_raw = R^T @ [0,0,-mg] + f_bias
    t_raw = CoM × (R^T @ [0,0,-mg]) + t_bias
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from fr3_stack import Robot

from ._common import (
    DEFAULT_HOST,
    collect_pose,
    default_calib_path,
    load_mount_rpy,
    print_result,
    rpy_to_R,
    save_yaml,
    solve,
)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default=DEFAULT_HOST,
                   help=f"NUC host or IP running fr3d (default {DEFAULT_HOST})")
    p.add_argument("--calib", type=Path, default=default_calib_path(),
                   help="output YAML path (default %(default)s)")
    p.add_argument("--rpy-ee-sensor", nargs=3, type=float, metavar=("R", "P", "Y"),
                   default=None,
                   help="EE→sensor rotation (radians, ZYX). Default reads from "
                        "existing --calib, else identity. With a Desk hand, pass "
                        "'0 0 0.7854' to undo Desk's -45° z-twist.")
    args = p.parse_args()

    rpy_ee_sensor = (np.asarray(args.rpy_ee_sensor, dtype=float)
                     if args.rpy_ee_sensor is not None
                     else load_mount_rpy(args.calib))
    R_ee_sensor = rpy_to_R(rpy_ee_sensor)

    with Robot(args.host) as robot:
        print(f"connecting to fr3d at {args.host} …")
        s = robot.wait_for_state(timeout=5.0)
        if not s.has_ft_sensor:
            print("ERROR: daemon is not publishing wrench_ft. Start fr3d with "
                  "--ft-sensor-kind bota --ft-sensor-config <path>.")
            return

        bar = "=" * 56
        print(bar)
        print("FT PAYLOAD CALIBRATION")
        print(bar)
        print(f"  rpy_ee_sensor = [{rpy_ee_sensor[0]:+.4f}, {rpy_ee_sensor[1]:+.4f}, "
              f"{rpy_ee_sensor[2]:+.4f}] rad" +
              (" (identity)" if not np.any(rpy_ee_sensor) else ""))
        print("Move the arm to a static pose, hold still, press Enter to record.")
        print("Vary the EE tilt (sensor pointing up / down / sideways / 45°…).")
        print("Aim for >=6 poses. Press 'q' Enter to solve.")
        print(bar)

        poses = []
        while True:
            ans = input(f"\npose {len(poses) + 1} — Enter to record, q to solve: ").strip()
            if ans.lower() == "q":
                break
            print("  recording (hold still)…")
            res = collect_pose(robot, R_ee_sensor=R_ee_sensor)
            if res is not None:
                poses.append(res)
                print(f"  recorded #{len(poses)}")

        result = solve(poses)
        if result is None:
            return
        # Round-trip so daemon's load_payload_calib uses the same R_ee_sensor.
        result["rpy_ee_sensor"] = [float(v) for v in rpy_ee_sensor]

        print_result(result)
        save_yaml(result, args.calib)
        print(f"\nsaved → {args.calib}")


if __name__ == "__main__":
    main()
