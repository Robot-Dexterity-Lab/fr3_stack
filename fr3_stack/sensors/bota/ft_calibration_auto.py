"""Automated FT calibration: replay recorded waypoints, average FT at each,
solve for payload. First waypoint is the start/return pose. Run
``fr3-ft-calibrate-record`` first to capture waypoints.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from fr3_stack import Arm, Pose

from ._common import (
    DEFAULT_HOST,
    collect_pose,
    default_calib_path,
    default_waypoints_path,
    load_mount_rpy,
    print_result,
    rpy_to_R,
    save_yaml,
    solve,
)

# Stiffer than the smoke.py K=200 so the arm holds against a tool payload.
SETTLE_TIME   = 2.0
MOVE_DURATION = 5.0
CALIB_K       = [400.0] * 3 + [30.0] * 3
CALIB_DAMP    = 0.9


def load_waypoints(path: Path) -> tuple[list[Pose] | None, str | None]:
    """Read Cartesian targets from a waypoints YAML.

    Bails with a clear message if pos/quat_xyzw are missing — we don't pull
    in pinocchio just to FK from joint angles.
    """
    if not path.exists():
        return None, (
            f"waypoints file not found: {path}\n"
            f"  run `fr3-ft-calibrate-record [host]` first."
        )
    with open(path) as fp:
        data = yaml.safe_load(fp)
    if not isinstance(data, dict):
        return None, f"invalid waypoints in {path}: expected a YAML mapping"
    waypoints = data.get("waypoints")
    if not waypoints:
        return None, (
            f"no 'waypoints' list in {path} — the schema changed; "
            f"re-record with `fr3-ft-calibrate-record [host]`."
        )
    poses: list[Pose] = []
    missing: list[int] = []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            return None, f"waypoint #{i + 1} in {path} is not a mapping"
        pos  = wp.get("pos")
        quat = wp.get("quat_xyzw")
        if pos is None or quat is None or len(pos) != 3 or len(quat) != 4:
            missing.append(i + 1)
            continue
        poses.append(Pose(np.asarray(pos,  dtype=float),
                          np.asarray(quat, dtype=float)))
    if missing:
        return None, (
            f"waypoints {missing} in {path} are missing pos/quat_xyzw — "
            "this Cartesian-impedance script needs both for every pose. "
            "Re-run `fr3-ft-calibrate-record` so each waypoint captures "
            "the live EE pose."
        )
    return poses, None


def fmt_pos(p: np.ndarray) -> str:
    return f"[{p[0]*1000:+7.1f}, {p[1]*1000:+7.1f}, {p[2]*1000:+7.1f}] mm"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default=DEFAULT_HOST,
                   help=f"NUC host or IP running fr3d (default {DEFAULT_HOST})")
    p.add_argument("--calib", type=Path, default=default_calib_path(),
                   help="output YAML path (default %(default)s)")
    p.add_argument("--waypoints", type=Path, default=default_waypoints_path(),
                   help="input waypoints YAML (default %(default)s)")
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

    poses_to_visit, err = load_waypoints(args.waypoints)
    if poses_to_visit is None:
        print(err); sys.exit(1)
    start = poses_to_visit[0]

    bar = "=" * 60
    print(bar)
    print("AUTOMATED FT SENSOR PAYLOAD CALIBRATION")
    print(bar)
    print(f"  loaded {len(poses_to_visit)} waypoints from: {args.waypoints}")
    print(f"  start/return pose: {fmt_pos(start.pos)}")
    print(f"  rpy_ee_sensor:    [{rpy_ee_sensor[0]:+.4f}, {rpy_ee_sensor[1]:+.4f}, "
          f"{rpy_ee_sensor[2]:+.4f}] rad" +
          (" (identity)" if not np.any(rpy_ee_sensor) else ""))
    print(f"  move duration: {MOVE_DURATION}s/pose, settle: {SETTLE_TIME}s")
    print(f"  stiffness K = {CALIB_K}, damp_ratio = {CALIB_DAMP}")
    print( "  CLEAR THE WORKSPACE AROUND THE ARM NOW.")
    print(bar)
    try:
        input("Press Enter to start (Ctrl-C to abort)... ")
    except KeyboardInterrupt:
        print("\naborted."); return

    print(f"connecting to fr3d at {args.host} …")
    with Arm(args.host) as arm:
        # Drop through to Robot for state inspection — Arm.observe doesn't expose has_ft.
        s = arm.robot.wait_for_state(timeout=5.0)
        if not s.has_ft_sensor:
            print("ERROR: daemon is not publishing wrench_ft.")
            print("  start fr3d with --ft-sensor-kind bota --ft-sensor-config <path>.")
            return

        # Lock current pose before the first move_to — otherwise the move
        # would jolt from zero-torque idle into impedance.
        arm.set_stiffness(K=CALIB_K, damp_ratio=CALIB_DAMP)
        arm.hold()
        time.sleep(0.5)

        recorded: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        try:
            for i, target in enumerate(poses_to_visit, start=1):
                print(f"\n[{i}/{len(poses_to_visit)}] moving to "
                      f"{fmt_pos(target.pos)} ...")
                obs = arm.move_to(target, duration=MOVE_DURATION)
                err_mm = float(np.linalg.norm(obs.pose.pos - target.pos)) * 1000
                print(f"  arrived at {fmt_pos(obs.pose.pos)}  (Δ {err_mm:.1f} mm)")
                print(f"  settling for {SETTLE_TIME}s ...")
                time.sleep(SETTLE_TIME)
                res = collect_pose(arm.robot, R_ee_sensor=R_ee_sensor)
                if res is not None:
                    recorded.append(res)
            print(f"\nreturning to start pose {fmt_pos(start.pos)} ...")
            arm.move_to(start, duration=MOVE_DURATION)
        except KeyboardInterrupt:
            print("\ninterrupted during motion. solving with collected poses ...")

        arm.relax()

    result = solve(recorded)
    if result is None:
        return
    # Round-trip so daemon's load_payload_calib uses the same R_ee_sensor.
    result["rpy_ee_sensor"] = [float(v) for v in rpy_ee_sensor]

    print_result(result)
    save_yaml(result, args.calib)
    print(f"\nsaved → {args.calib}")


if __name__ == "__main__":
    main()
