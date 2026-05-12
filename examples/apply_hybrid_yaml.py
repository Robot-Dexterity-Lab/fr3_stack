"""Apply hybrid force-position control using fr3_stack/configs/hybrid.yaml.

Captures live pose as the position target and sends hybrid with all gains
from YAML (loaded into Robot._hybrid_cache). n_af=0 → pure admittance.

Usage:
    ./fr3-stack up hybrid -d --ft     # in another terminal, on the NUC
    python examples/apply_hybrid_yaml.py <host>
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from fr3_stack import Robot


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host", nargs="?", default="localhost")
    p.add_argument("--rate", type=float, default=20.0, help="state poll rate Hz (default 20)")
    p.add_argument("--no-ft", action="store_true", help="use libfranka estimate instead of FT sensor (debug)")
    p.add_argument("--unsafe", action="store_true", help="skip 3-second countdown")
    args = p.parse_args()

    require_ft = not args.no_ft

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state()
        c = robot._hybrid_cache
        print(f"connected.  pos0={np.round(s0.pos, 4)}  has_ft={s0.has_ft_sensor}")
        print("hybrid.yaml values being applied:")
        print(f"  n_af              = {c['n_af']}")
        print(f"  target_wrench_Tr  = {c['target_wrench_Tr']}")
        print(f"  inner  M/K/D_adm  = {c['M_adm']} / {c['K_adm']} / {c['D_adm']}")
        print(f"  force PID trans   = P={c['P_trans']:.3g}  I={c['I_trans']:.3g}  D={c['D_trans']:.3g}")
        print(f"  force PID rot     = P={c['P_rot']:.3g}  I={c['I_rot']:.3g}  D={c['D_rot']:.3g}")
        print(f"  stiction          = {c['stiction']}")
        print(f"  spring clip       = F<{c['max_spring_force']}N  τ<{c['max_spring_torque']}Nm")
        print(f"  outer  K / D      = {c['K']} / {c['D']}")
        print(f"  nullspace K/D/τmax= {c['K_null']} / {c['D_null']} / {c['max_tau_null']}")
        print(f"  α_pose / α_wrench = {c['filter_alpha']} / {c['wrench_filter_alpha']}")
        print(f"  use_friction      = {c['use_friction']}")

        if require_ft and not s0.has_ft_sensor:
            print("\nno FT sensor on state.wrench_ft — start daemon with --ft, "
                  "or pass --no-ft for libfranka estimate.")
            return

        target_pos  = s0.pos.copy()
        target_quat = s0.quat_xyzw.copy()

        if not args.unsafe:
            for i in range(3, 0, -1):
                print(f"  activating in {i}s ... (Ctrl+C to abort)", end="\r", flush=True)
                time.sleep(1.0)
            print(" " * 60, end="\r")

        try:
            robot.send_hybrid(
                target_pos        = target_pos,
                target_quat_xyzw  = target_quat,
                require_ft_sensor = require_ft,
            )
        except RuntimeError as e:
            print(f"\nactivation failed: {e}")
            return

        dt = 1.0 / args.rate
        peak_drift = 0.0
        print("active — Ctrl+C to stop.\n")
        try:
            while True:
                s = robot.state
                err   = s.pos - target_pos
                drift = float(np.linalg.norm(err))
                peak_drift = max(peak_drift, drift)

                F     = s.wrench_ft if s.has_ft_sensor else s.wrench_ext
                F_src = "ft" if s.has_ft_sensor else "lf"

                print(f"  ctrl={s.controller:12s}  "
                      f"err(mm)={np.round(err*1000, 1)}  "
                      f"|err|={drift*1000:5.1f}mm  "
                      f"F_{F_src}={np.round(F[:3], 2)}  "
                      f"M_{F_src}={np.round(F[3:], 2)}")
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\ninterrupted — sending idle ...")

        print(f"\npeak drift during test: {peak_drift*1000:.1f} mm")
        robot.send_idle()
        time.sleep(0.1)
        print("idled.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
