"""End-to-end command latency: Python.send_* → daemon RT switch → state echo.

Measures the round trip a command actually goes through:
  1. Python serializes Command (capnp) and pushes to ZMQ
  2. NUC's PULL socket receives, cmd_thread parses
  3. RT thread try_locks pending, switches active controller
  4. State publisher publishes new `controller` name
  5. Workstation SUB socket receives, decodes, fr3.state updates

We probe by alternating idle <-> cartesian_impedance and timing how long
until `robot.state.controller` reflects the new mode. Targets are pinned to
the current pose, so the arm physically stays still — only the controller
label flips.

Usage:
    python examples/bench_cmd.py 192.168.1.8 --n 100
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from fr3_stack import Robot


def wait_for_controller(robot: Robot, want: str, deadline: float) -> float | None:
    while time.monotonic() < deadline:
        if robot.state.controller == want:
            return time.monotonic()
        time.sleep(0.0005)            # 0.5 ms tight poll
    return None


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("--n", type=int, default=100,
                   help="round trips to measure (each = idle->cart->idle = 2)")
    p.add_argument("--timeout", type=float, default=0.5,
                   help="per-edge wait timeout in seconds")
    args = p.parse_args()

    latencies: list[float] = []         # ms, all successful edges

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state(timeout=5.0)
        print(f"connected.  starting controller={s0.controller}")

        # Anchor the cart_imp target to the current pose so the arm doesn't move.
        anchor_pos  = s0.pos.copy()
        anchor_quat = s0.quat_xyzw.copy()

        # Make sure we start in idle for a clean baseline.
        robot.send_idle()
        if wait_for_controller(robot, "idle", time.monotonic() + 1.0) is None:
            print("warn: never reached idle baseline; aborting")
            return

        misses = 0
        for i in range(args.n):
            # idle -> cartesian_impedance
            t_send = time.monotonic()
            robot.send_cartesian_impedance(
                target_pos       = anchor_pos,
                target_quat_xyzw = anchor_quat,
            )
            t_recv = wait_for_controller(
                robot, "cartesian_impedance", t_send + args.timeout)
            if t_recv is None: misses += 1
            else:              latencies.append((t_recv - t_send) * 1e3)

            # cartesian_impedance -> idle
            t_send = time.monotonic()
            robot.send_idle()
            t_recv = wait_for_controller(
                robot, "idle", t_send + args.timeout)
            if t_recv is None: misses += 1
            else:              latencies.append((t_recv - t_send) * 1e3)

    if not latencies:
        print("no successful round trips — daemon unreachable or rejecting commands?")
        return

    a = np.array(latencies)
    print()
    print(f"samples         {len(a)} edges over {args.n} round trips "
          f"({misses} misses)")
    print(f"latency (ms)    "
          f"p50 {np.median(a):.2f}   "
          f"p90 {np.percentile(a, 90):.2f}   "
          f"p99 {np.percentile(a, 99):.2f}   "
          f"max {a.max():.2f}")
    print(f"             min {a.min():.2f}   mean {a.mean():.2f}   "
          f"stddev {a.std():.2f}")
    print()
    print("note: floor ≈ ½ state-publish period (2.5 ms @ 200 Hz) plus ~1 ms")
    print("      RT tick latency. Sub-10 ms p99 is healthy on a host network.")


if __name__ == "__main__":
    main()
