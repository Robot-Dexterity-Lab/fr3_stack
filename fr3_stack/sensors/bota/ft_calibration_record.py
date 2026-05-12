"""Record waypoints for ``fr3-ft-calibrate-auto``.

Daemon idle is zero-torque so the arm hand-guides freely. First recorded pose
is the start/return pose. Aim for ≥10 diverse poses.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from fr3_stack import Robot

from ._common import DEFAULT_HOST, default_waypoints_path, save_yaml

JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default=DEFAULT_HOST,
                   help=f"NUC host or IP running fr3d (default {DEFAULT_HOST})")
    p.add_argument("--waypoints", type=Path, default=default_waypoints_path(),
                   help="output waypoints YAML (default %(default)s)")
    args = p.parse_args()

    with Robot(args.host) as robot:
        print(f"connecting to fr3d at {args.host} …")
        robot.wait_for_state(timeout=5.0)

        robot.send_idle()
        print("\n  daemon set to idle (zero-torque gravity comp) — free-drive enabled.")

        bar = "=" * 60
        print(bar)
        print("RECORD CALIBRATION WAYPOINTS")
        print(bar)
        print("  Move the arm by hand to each pose, press Enter to record.")
        print("  The FIRST pose is the start/return pose.")
        print("  Aim for >=10 diverse poses (wrist up/down/side, forearm rolled).")
        print("  Type 'q' + Enter when done.")
        print(bar)

        waypoints: list[dict] = []
        try:
            while True:
                ans = input(
                    f"\npose {len(waypoints) + 1} — Enter to record, 'q' to save & quit: "
                ).strip().lower()
                if ans == "q":
                    break
                s = robot.state
                if not s.valid:
                    print("  no state from daemon yet; try again.")
                    continue
                wp = {
                    "q":         [float(v) for v in s.q],
                    "pos":       [float(v) for v in s.pos],
                    "quat_xyzw": [float(v) for v in s.quat_xyzw],
                }
                waypoints.append(wp)
                print("  joints: " + ", ".join(f"{v:+.4f}" for v in wp["q"]))
                print("  ee pos: " + ", ".join(f"{v:+.4f}" for v in wp["pos"]))
                print("  ee quat (xyzw): " + ", ".join(f"{v:+.4f}" for v in wp["quat_xyzw"]))
        except KeyboardInterrupt:
            print("\ninterrupted.")

    if not waypoints:
        print("no waypoints recorded, nothing saved.")
        return

    save_yaml(
        {"joint_names": JOINT_NAMES, "waypoints": waypoints},
        args.waypoints,
    )
    print(f"\nsaved {len(waypoints)} waypoints → {args.waypoints}")
    print("  first pose is the start/return pose.")
    print("  next: fr3-ft-calibrate-auto")


if __name__ == "__main__":
    main()
