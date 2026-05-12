"""Interactive API smoke test: can I move the arm? can I set its parameters?

Walks the operator through each major Arm primitive in order:
    observe → relax → hold → move_to → set_stiffness → streaming → profile

Each step prints what's about to happen, counts down 3 s (Ctrl+C aborts),
runs the op, then waits for Enter before continuing. Designed to be re-run
any time you want to confirm the daemon + Python client are healthy.

Prereq: NUC daemon up. No FT sensor required.

Usage:
    uv run python examples/smoke.py <nuc-host>
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from fr3_stack import Arm, Pose


SAFE_MAX_DELTA = 0.08    # ±8 cm hard cap on any move


def banner(title: str) -> None:
    print(f"\n{'='*64}\n  {title}\n{'='*64}")


def countdown(secs: int = 3, msg: str = "running in") -> None:
    for i in range(secs, 0, -1):
        print(f"  {msg} {i}s ... (Ctrl+C aborts)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 64, end="\r")


def wait(prompt: str = "  press Enter to continue ") -> None:
    try:
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        print("\n  aborted by operator")
        sys.exit(0)


def fmt_pos(p: np.ndarray) -> str:
    return f"[{p[0]*1000:+7.1f}, {p[1]*1000:+7.1f}, {p[2]*1000:+7.1f}] mm"


def step_observe(arm: Arm) -> Pose:
    banner("1/7  observe — read current state")
    obs = arm.observe()
    print(f"  pos       = {fmt_pos(obs.pose.pos)}")
    print(f"  quat xyzw = {np.round(obs.pose.quat, 3)}")
    print(f"  q (rad)   = {np.round(obs.q, 3)}")
    print(f"  has_ft    = {obs.has_ft}")
    print(f"  wrench    = {np.round(obs.wrench, 2)} (N, Nm)")
    wait()
    return obs.pose


def step_relax(arm: Arm) -> None:
    banner("2/7  relax — gravity comp (free-drive)")
    print("  arm should now be hand-guidable. Try moving it gently.")
    arm.relax()
    wait("  done feeling, press Enter to continue ")


def step_hold(arm: Arm) -> None:
    banner("3/7  hold — lock at current pose")
    print("  arm should resist when you push. Default K=200 N/m.")
    arm.hold()
    wait("  pushed it? press Enter to continue ")


def step_move_to(arm: Arm, anchor: Pose) -> None:
    banner("4/7  move_to — min-jerk +5 cm Z, then back")
    target_up = Pose(anchor.pos + np.array([0, 0, 0.05]), anchor.quat.copy())

    print(f"  from {fmt_pos(anchor.pos)}")
    print(f"  to   {fmt_pos(target_up.pos)}  (Δz = +50 mm, T=2 s)")
    countdown()
    obs = arm.move_to(target_up, duration=2.0)
    err_mm = np.linalg.norm(obs.pose.pos - target_up.pos) * 1000
    print(f"  arrived at {fmt_pos(obs.pose.pos)}")
    print(f"  position error: {err_mm:.2f} mm")

    print(f"\n  returning to anchor in 2 s ...")
    countdown()
    obs2 = arm.move_to(anchor, duration=2.0)
    err2_mm = np.linalg.norm(obs2.pose.pos - anchor.pos) * 1000
    print(f"  back at {fmt_pos(obs2.pose.pos)}, error {err2_mm:.2f} mm")
    wait()


def step_stiffness(arm: Arm) -> None:
    banner("5/7  set_stiffness — push test, soft vs stiff")
    arm.hold()
    levels = [
        ("soft",   [80,  80,  80,   8,  8,  8],  0.9),
        ("medium", [200, 200, 200, 20, 20, 20],  0.9),
        ("stiff",  [600, 600, 600, 40, 40, 40],  0.9),
    ]
    for name, K, ratio in levels:
        arm.set_stiffness(K=K, damp_ratio=ratio)
        # set_stiffness is sticky but doesn't push to the daemon by itself —
        # send one cmd with current pose to make the new K take effect now.
        arm.hold()
        D = (2.0 * np.sqrt(np.asarray(K, dtype=float)) * ratio).round(1).tolist()
        print(f"\n  --- {name} ---")
        print(f"  K = {K}")
        print(f"  D = {D}")
        print("  push the EE in any direction — feel how much it gives.")
        wait("  felt? press Enter for next level ")
    print("\n  resetting to default K = [200,200,200,20,20,20] D = [28,...]")
    arm.set_stiffness(K=[200]*3 + [20]*3, D=[28]*3 + [9]*3)
    arm.hold()
    wait()


def step_streaming(arm: Arm, anchor: Pose) -> None:
    banner("6/7  send (streaming) — 3 cm circle in XY, 4 s, 100 Hz")
    arm.move_to(anchor, duration=1.5)
    radius = 0.03
    period = 4.0
    rate   = 100.0
    n      = int(period * rate)

    countdown()
    t0 = time.monotonic()
    errs = []
    for i in range(n):
        t   = (time.monotonic() - t0)
        ang = 2 * np.pi * t / period
        target = Pose(
            anchor.pos + np.array([radius * (np.cos(ang) - 1),
                                   radius * np.sin(ang),
                                   0.0]),
            anchor.quat.copy(),
        )
        arm.send(target)
        if i % 10 == 0:
            obs = arm.observe()
            errs.append(np.linalg.norm(obs.pose.pos - target.pos) * 1000)
        time.sleep(max(0.0, t0 + (i + 1) / rate - time.monotonic()))
    print(f"  loop done. mean tracking err = {np.mean(errs):.2f} mm, "
          f"max = {np.max(errs):.2f} mm")

    print("  returning to anchor ...")
    arm.move_to(anchor, duration=1.5)
    wait()


def step_profile(arm: Arm) -> None:
    banner("7/7  use_profile — reload K/D/etc from a YAML profile variant")
    # Profile variants live at configs/cartesian_impedance.<name>.yaml, with
    # the base configs/cartesian_impedance.yaml being the no-profile case.
    # Auto-discover any variants the user has defined.
    from pathlib import Path
    cfg_dir   = Path(__file__).resolve().parent.parent / "fr3_stack" / "configs"
    variants  = sorted(p.stem.split(".", 1)[1]
                       for p in cfg_dir.glob("cartesian_impedance.*.yaml"))
    if not variants:
        print(f"  no profile variants found in {cfg_dir}")
        print(f"  (only the base cartesian_impedance.yaml exists — already loaded)")
        print(f"  to test profile switching, create a variant like")
        print(f"    {cfg_dir}/cartesian_impedance.stiff.yaml")
        print(f"  then re-run this script.")
        wait("  press Enter to skip ")
        return

    print(f"  available variants: {variants}")
    target = variants[0]
    print(f"  switching to '{target}' (configs/cartesian_impedance.{target}.yaml)")
    arm.use_profile(target)
    arm.hold()
    print(f"  K/D/etc reloaded. push to confirm.")
    wait()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("host", nargs="?", default="localhost",
                    help="NUC host or IP (default: localhost)")
    ap.add_argument("--skip-move", action="store_true",
                    help="skip move_to / streaming (do only stand-still tests)")
    args = ap.parse_args()

    print(f"connecting to {args.host} ...")
    with Arm(args.host) as arm:
        try:
            anchor = step_observe(arm)
            step_relax(arm)
            step_hold(arm)
            if not args.skip_move:
                step_move_to(arm, anchor)
            step_stiffness(arm)
            if not args.skip_move:
                step_streaming(arm, anchor)
            step_profile(arm)

            banner("done — releasing to gravity comp")
            arm.relax()
            print("  smoke test passed. arm is in free-drive.")
        except KeyboardInterrupt:
            print("\n\naborted by operator. releasing to gravity comp.")
            arm.relax()
            sys.exit(1)
        except Exception as e:
            print(f"\n\nERROR: {e}")
            print("releasing to gravity comp.")
            arm.relax()
            raise


if __name__ == "__main__":
    main()
