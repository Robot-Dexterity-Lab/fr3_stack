"""Measure how fast the workstation can read fr3 state.

Polls `robot.state` at the requested rate for `--secs` seconds and reports:
  - achieved rate          (actual samples / second)
  - unique daemon samples  (timestamp changes — daemon publishes ~200 Hz)
  - latency                (now - daemon_timestamp), p50/p99
  - jitter                 (delta-time stddev around the target period)

Usage:
    python examples/bench_state.py 192.168.1.8 --hz 30 --secs 5
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from fr3_stack import Robot


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("host", nargs="?", default="localhost",
                   help="NUC host or IP (default: localhost)")
    p.add_argument("--hz",   type=float, default=30.0, help="target poll rate")
    p.add_argument("--secs", type=float, default=5.0,  help="how long to sample")
    args = p.parse_args()

    period = 1.0 / args.hz
    n      = int(args.hz * args.secs)
    print(f"polling {args.host} @ {args.hz} Hz for {args.secs}s "
          f"({n} samples)")

    poll_t   = np.zeros(n)
    daemon_t = np.zeros(n)

    with Robot(args.host) as robot:
        s0 = robot.wait_for_state(timeout=5.0)
        print(f"connected.  daemon controller={s0.controller}  running={s0.running}")

        # Sleep-to-deadline so we measure poll cadence, not "tight loop".
        t0 = time.monotonic()
        for i in range(n):
            deadline = t0 + (i + 1) * period
            s = robot.state
            poll_t[i]   = time.monotonic()
            daemon_t[i] = s.timestamp
            slack = deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
        t_end = time.monotonic()

    dt = np.diff(poll_t)
    actual_hz = (n - 1) / (poll_t[-1] - poll_t[0])
    n_unique  = len(np.unique(daemon_t))
    latency   = (poll_t - daemon_t) * 1e3   # ms

    print()
    print(f"target rate     {args.hz:.1f} Hz   ({period*1e3:.2f} ms period)")
    print(f"achieved rate   {actual_hz:.2f} Hz")
    print(f"unique daemon   {n_unique}/{n} samples"
          f"  ({100*n_unique/n:.0f}% — daemon publishes ~200 Hz)")
    print(f"period jitter   mean dt {dt.mean()*1e3:.2f} ms   "
          f"stddev {dt.std()*1e3:.2f} ms   "
          f"max {dt.max()*1e3:.2f} ms")
    print(f"latency (now-stamp)   p50 {np.median(latency):.2f}   "
          f"p99 {np.percentile(latency, 99):.2f}   "
          f"max {latency.max():.2f}  ms")


if __name__ == "__main__":
    main()
