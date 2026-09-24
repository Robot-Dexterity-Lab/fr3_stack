#!/usr/bin/env python3
"""Preview or stream bounded joint-space excitation to one fr3-stack daemon.

Default: offline preview, no Robot import or network connection. See
docs/joint-sysid.md for command examples, data semantics, and hardware limits.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np


# Match include/fr3_stack/utils/controllers_common.hpp, including the 10%
# repulsion region. These checks do not establish collision-free motion.
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, .5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -.1518, 2.8065, 4.5169, 3.0159])
Q_LOWER = Q_MIN + .1 * (Q_MAX - Q_MIN)
Q_UPPER = Q_MAX - .1 * (Q_MAX - Q_MIN)
PREVIEW_Q = np.array([0., -.6, 0., -1.8, 0., 1.8, .7])
COLORS = ['#2563eb', '#dc2626', '#16a34a', '#9333ea', '#ea580c', '#0891b2', '#475569']


@dataclass(frozen=True)
class Plan:
    mode: str = 'sine'
    joints: tuple[int, ...] = (1,)
    amplitude_deg: float = 1.0
    seconds: float = 20.0
    ramp: float = 5.0
    hold: float = 2.0
    gap: float = 2.0
    f0: float = .1
    f1: float = .5
    tones: tuple[float, ...] = (.1, .23, .41)
    seed: int = 0
    rate: float = 60.0
    max_velocity: float = .1
    max_acceleration: float = .3
    max_jerk: float = 2.0

    @property
    def segments(self):
        return 1 if self.mode in ('hold', 'multisine') else len(self.joints)

    @property
    def total(self):
        return 2 * self.hold + self.segments * self.seconds + (self.segments - 1) * self.gap

    def validate(self):
        values = [self.amplitude_deg, self.seconds, self.ramp, self.hold, self.gap,
                  self.f0, self.f1, self.rate, self.max_velocity,
                  self.max_acceleration, self.max_jerk, *self.tones]
        if not np.isfinite(values).all():
            raise ValueError('all trajectory settings must be finite')
        if self.mode not in ('hold', 'sine', 'chirp', 'multisine'):
            raise ValueError('unknown trajectory mode')
        if not self.joints or len(set(self.joints)) != len(self.joints) or any(j not in range(1, 8) for j in self.joints):
            raise ValueError('joints must be unique indices in 1..7')
        if self.amplitude_deg <= 0 or self.seconds <= 0 or self.ramp <= 0 or 2 * self.ramp > self.seconds:
            raise ValueError('positive amplitude/duration/ramp required; 2*ramp <= seconds')
        if self.hold < 0 or self.gap < 0 or self.total > 600:
            raise ValueError('hold/gap must be nonnegative; total duration must be <= 600 s')
        if self.f0 <= 0 or self.f1 < self.f0 or not self.tones or min(self.tones) <= 0:
            raise ValueError('positive frequencies required and f1 >= f0')
        if self.seed < 0:
            raise ValueError('seed must be nonnegative')
        if not 10 <= self.rate <= 200:
            raise ValueError('command rate must be in 10..200 Hz')
        if self.ramp * self.rate < 5:
            raise ValueError('each ramp must span at least five command periods')
        top = max(self.tones) if self.mode == 'multisine' else self.f1 if self.mode == 'chirp' else self.f0
        if self.mode != 'hold' and top > self.rate / 20:
            raise ValueError('use at least 20 command samples per highest-frequency cycle')
        if min(self.max_velocity, self.max_acceleration, self.max_jerk) <= 0:
            raise ValueError('derivative limits must be positive')


def envelope(t, duration, ramp):
    """C3 fade-in/out; return w and its first three time derivatives."""
    # Seventh-order smoothstep: all three derivatives vanish at each end.
    coefficients = [0., 0., 0., 0., 35., -84., 70., -20.]
    t = np.asarray(t, dtype=float)
    u = np.clip(np.minimum(t, duration - t) / ramp, 0, 1)
    direction = np.where(t <= duration / 2, 1., -1.)
    result = []
    for order in range(4):
        c = np.polynomial.polynomial.polyder(coefficients, order)
        value = np.polynomial.polynomial.polyval(u, c) * (direction / ramp) ** order
        if order:
            value = np.where((u > 0) & (u < 1), value, 0.)
        result.append(value)
    return result


class Trajectory:
    def __init__(self, plan: Plan):
        plan.validate()
        self.plan = plan
        self.phases = np.random.default_rng(plan.seed).uniform(0, 2 * np.pi, (7, len(plan.tones)))

    def evaluate(self, times):
        """Return offsets, velocity, acceleration, jerk, each shaped (N, 7).

        These are derivatives of the continuous reference. The actual network
        commands are sampled, held by the daemon, and filtered by its controller.
        """
        t = np.atleast_1d(np.asarray(times, dtype=float))
        p = self.plan
        output = [np.zeros((len(t), 7)) for _ in range(4)]
        if p.mode == 'hold':
            return output
        for segment in range(p.segments):
            start = p.hold + segment * (p.seconds + p.gap)
            active = (t >= start) & (t <= start + p.seconds)
            if not active.any():
                continue
            s = t[active] - start
            w, dw, ddw, dddw = envelope(s, p.seconds, p.ramp)
            joints = p.joints if p.mode == 'multisine' else (p.joints[segment],)
            for joint in joints:
                y = [np.zeros_like(s) for _ in range(4)]
                tones = p.tones if p.mode == 'multisine' else (p.f0,)
                # Normalize the SUM of tone amplitudes, not each tone, to A.
                weights = 1 / np.asarray(tones)
                weights /= weights.sum()
                for k, (frequency, weight) in enumerate(zip(tones, weights)):
                    sweep = (p.f1 - p.f0) / p.seconds if p.mode == 'chirp' else 0.
                    phase0 = self.phases[joint - 1, k] if p.mode == 'multisine' else 0.
                    phase = 2 * np.pi * (frequency * s + .5 * sweep * s**2) + phase0
                    omega = 2 * np.pi * (frequency + sweep * s)
                    beta = 2 * np.pi * sweep
                    a = np.deg2rad(p.amplitude_deg) * weight
                    sn, cs = np.sin(phase), np.cos(phase)
                    y[0] += a * sn
                    y[1] += a * omega * cs
                    y[2] += a * (beta * cs - omega**2 * sn)
                    y[3] += a * (-3 * omega * beta * sn - omega**3 * cs)
                output[0][active, joint - 1] = w * y[0]
                output[1][active, joint - 1] = dw * y[0] + w * y[1]
                output[2][active, joint - 1] = ddw * y[0] + 2 * dw * y[1] + w * y[2]
                output[3][active, joint - 1] = dddw * y[0] + 3 * ddw * y[1] + 3 * dw * y[2] + w * y[3]
        return output

    def check(self, q0):
        q0 = vector(q0, 'anchor')
        p = self.plan
        # A conservative position bound covers between-sample peaks as well.
        excursion = np.zeros(7)
        if p.mode != 'hold':
            excursion[np.array(p.joints) - 1] = np.deg2rad(p.amplitude_deg)
        if np.any(q0 - excursion <= Q_LOWER) or np.any(q0 + excursion >= Q_UPPER):
            raise ValueError('anchor/excursion enters the FR3 joint-limit repulsion region')
        t = np.linspace(0, p.total, int(np.ceil(p.total * 1000)) + 1)
        values = self.evaluate(t)
        peaks = [np.max(np.abs(x), axis=0) for x in values]
        for name, peak, limit in zip(('velocity', 'acceleration', 'jerk'), peaks[1:],
                                     (p.max_velocity, p.max_acceleration, p.max_jerk)):
            if np.any(peak > limit):
                raise ValueError(f'reference {name} peak {max(peak):.6g} exceeds {limit}; reduce amplitude/frequency or lengthen ramps')
        return {name: peak.tolist() for name, peak in zip(
            ('offset_rad', 'velocity_rad_s', 'acceleration_rad_s2', 'jerk_rad_s3'), peaks)}


def vector(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (7,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must contain seven finite values')
    return a


def write_json(path, data):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def write_preview(output, trajectory, q0):
    p = trajectory.plan
    t = np.linspace(0, p.total, int(np.ceil(p.total * p.rate)) + 1)
    values = trajectory.evaluate(t)
    with (output / 'reference.csv').open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_s'] + [f'{key}{j}' for key in ('q_ref', 'dq_ref', 'ddq_ref', 'jerk_ref') for j in range(1, 8)])
        writer.writerows(np.column_stack([t, values[0] + q0, *values[1:]]))
    # Standalone SVG keeps the preview usable without a plotting dependency.
    indices = np.unique(np.linspace(0, len(t) - 1, min(len(t), 2000)).astype(int))
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="820" viewBox="0 0 1000 820">',
           '<rect width="1000" height="820" fill="white"/>',
           '<g font-family="sans-serif" font-size="13" fill="#111827">',
           '<text x="65" y="24">Joint reference preview (continuous reference; no collision or torque validation)</text>']
    for j, color in enumerate(COLORS):
        svg.append(f'<text x="{100 + j * 100}" y="46" fill="{color}">J{j + 1}</text>')
    labels = ('Offset (deg)', 'Velocity (rad/s)', 'Acceleration (rad/s^2)', 'Jerk (rad/s^3)')
    for panel, (label, value) in enumerate(zip(labels, values)):
        value = np.rad2deg(value) if panel == 0 else value
        top, height = 75 + 180 * panel, 130
        scale = max(float(np.max(np.abs(value))), 1e-6) * 1.1
        svg.append(f'<text x="65" y="{top - 9}">{label}; range +/- {scale:.4g}</text>')
        svg.append(f'<rect x="65" y="{top}" width="900" height="{height}" fill="none" stroke="#cbd5e1"/>')
        svg.append(f'<path d="M65 {top + height / 2}H965" stroke="#e2e8f0"/>')
        for j, color in enumerate(COLORS):
            points = ' '.join(f'{65 + 900 * t[k] / p.total:.2f},{top + height / 2 - value[k, j] * height / (2 * scale):.2f}' for k in indices)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.3"/>')
        svg.append(f'<text x="65" y="{top + height + 18}">0 s</text><text x="890" y="{top + height + 18}">{p.total:g} s</text>')
    svg.append('</g></svg>')
    (output / 'preview.svg').write_text('\n'.join(svg))


def healthy_state(state, now, *, stale_after, max_velocity):
    if not state.valid or state.received_at is None or not np.isfinite(state.received_at):
        raise RuntimeError('no valid timestamped state')
    age = now - state.received_at
    if age < 0 or age > stale_after:
        raise RuntimeError(f'stale state: age {age:.3f} s')
    if not state.running or state.last_error:
        raise RuntimeError(f'daemon not healthy: running={state.running}, error={state.last_error!r}')
    if state.controller != 'joint_impedance':
        raise RuntimeError('daemon must already be in joint_impedance mode')
    q, dq = vector(state.q, 'measured q'), vector(state.dq, 'measured dq')
    if not np.isfinite(state.timestamp):
        raise RuntimeError('nonfinite daemon timestamp')
    if np.any(q <= Q_LOWER) or np.any(q >= Q_UPPER):
        raise RuntimeError('measured joint position enters the repulsion region')
    if np.max(np.abs(dq)) > max_velocity:
        raise RuntimeError('measured joint speed exceeds configured bound')
    return state


def stream(robot, trajectory, q0, gains, writer, metadata, *, stale_after=.2,
           tracking_error=.05, max_lateness=.05, clock=time.monotonic, sleep=time.sleep):
    """Stream against wall time; abort on pauses instead of catching up in bursts."""
    p = trajectory.plan
    origin = clock()
    deadline = origin
    last_target = q0.copy()
    seq = 0
    slot = 0
    while True:
        sleep(max(0., deadline - clock()))
        snapshot = robot.state
        now = clock()
        lateness = now - deadline
        if lateness > max_lateness:
            raise RuntimeError(f'command loop missed deadline by {lateness:.3f} s')
        state = healthy_state(snapshot, now, stale_after=stale_after, max_velocity=p.max_velocity)
        if np.max(np.abs(state.q - last_target)) > tracking_error:
            raise RuntimeError('joint tracking error exceeds configured bound')
        elapsed = min(now - origin, p.total)
        target = q0 + trajectory.evaluate([elapsed])[0][0]
        # This is an attempted command; send completion is not a daemon ack.
        metadata['motion_command_attempted'] = True
        robot.send_joint_impedance(target, **gains)
        sent_at = clock()
        metadata['commands_sent'] = seq + 1
        writer.writerow([seq, elapsed, now, sent_at, state.timestamp, state.received_at,
                         lateness, *target, *state.q, *state.dq])
        if sent_at - now > max_lateness:
            raise RuntimeError('command send exceeded timing budget')
        if elapsed >= p.total:
            if np.max(np.abs(state.q - q0)) > tracking_error or np.max(np.abs(state.dq)) > .02:
                raise RuntimeError('arm did not settle at the anchor')
            break
        seq += 1
        # Skip expired slots. The next target is evaluated at actual wall time.
        slot = max(slot + 1, int(np.floor((sent_at - origin) * p.rate)) + 1)
        deadline = origin + slot / p.rate


def request_termination(robot, metadata):
    metadata['termination'] = 'attempted'
    try:
        robot.terminate()
        # Allow the queued message time to leave before close(linger=0).
        # A disappearing state stream is NOT confirmation of a stop.
        time.sleep(.1)
        metadata['termination'] = 'requested_unconfirmed'
        return True
    except Exception as exc:
        metadata['termination'] = f'request_failed: {exc}'
        return False


def run_live(robot, trajectory, gains, output, metadata, args):
    """Own the motion/error lifecycle; preserve partial logs and request stop."""
    try:
        state = healthy_state(robot.wait_for_state(timeout=5), time.monotonic(),
                              stale_after=args.stale_after, max_velocity=.02)
        q0 = state.q.copy()
        metadata['anchor_q_rad'] = q0.tolist()
        metadata['reference_peaks'] = trajectory.check(q0)
        write_preview(output, trajectory, q0)
        metadata['status'] = 'prepared'
        write_json(output / 'metadata.json', metadata)
        # Preview/file writes can take time. Do not start from an old anchor.
        state = healthy_state(robot.state, time.monotonic(), stale_after=args.stale_after, max_velocity=.02)
        if np.max(np.abs(state.q - q0)) > .002:
            raise RuntimeError('arm moved while preparing the trajectory; rerun from a stationary pose')
        with (output / 'commands_and_state.csv').open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['seq', 'trajectory_t_s', 'send_start_monotonic_s', 'send_return_monotonic_s',
                             'state_daemon_timestamp', 'state_received_monotonic_s', 'lateness_s'] +
                            [f'{key}{j}' for key in ('q_target', 'q_measured', 'dq_measured') for j in range(1, 8)])
            f.flush()
            metadata['status'] = 'running'
            write_json(output / 'metadata.json', metadata)
            print(f"Streaming {trajectory.plan.mode}: {trajectory.plan.total:g} s; output: {output}", flush=True)
            stream(robot, trajectory, q0, gains, writer, metadata,
                   stale_after=args.stale_after, tracking_error=args.tracking_error,
                   max_lateness=args.max_lateness)
        if args.finish == 'terminate':
            if not request_termination(robot, metadata):
                raise RuntimeError('trajectory finished but daemon termination request failed')
        metadata['status'] = 'completed'
        metadata['end_behavior'] = args.finish
    except BaseException as exc:
        metadata['status'] = 'aborted'
        metadata['error'] = f'{type(exc).__name__}: {exc}'
        if metadata.get('motion_command_attempted'):
            request_termination(robot, metadata)
        raise
    finally:
        metadata['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(output / 'metadata.json', metadata)


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='send commands to the specified NUC')
    mode.add_argument('--dry-run', action='store_true', help='explicit offline preview (the default)')
    p.add_argument('--host', help='NUC host; used only with --execute')
    p.add_argument('--cmd-port', type=int, default=5555)
    p.add_argument('--state-port', type=int, default=5556)
    p.add_argument('--robot-id', help='recording identifier, e.g. left or right; required for execution')
    p.add_argument('--mode', choices=['hold', 'sine', 'chirp', 'multisine'], default='sine')
    p.add_argument('--joints', type=int, nargs='+', default=[1], help='1-based; sine/chirp run sequentially')
    p.add_argument('--amplitude-deg', type=float, default=1., help='peak offset bound per selected joint, not peak-to-peak')
    p.add_argument('--seconds', type=float, default=20., help='duration per joint for sine/chirp; includes ramps')
    p.add_argument('--ramp', type=float, default=5., help='fade-in and fade-out duration each')
    p.add_argument('--hold', type=float, default=2., help='initial and final holding duration each')
    p.add_argument('--gap', type=float, default=2., help='hold between sequential joint segments')
    p.add_argument('--f0', type=float, default=.1, help='sine frequency / chirp starting frequency, Hz')
    p.add_argument('--f1', type=float, default=.5, help='chirp final frequency, Hz')
    p.add_argument('--tones', type=float, nargs='+', default=[.1, .23, .41], help='multisine frequencies, Hz')
    p.add_argument('--seed', type=int, default=0, help='reproducible multisine phases')
    p.add_argument('--rate', type=float, default=60., help='workstation target rate; NOT the NUC logging rate')
    p.add_argument('--q0-rad', type=float, nargs=7, help='offline preview anchor only; execution always uses live q')
    p.add_argument('--max-velocity', type=float, default=.1, help='reference and measured joint speed limit, rad/s')
    p.add_argument('--max-acceleration', type=float, default=.3, help='continuous reference limit, rad/s^2')
    p.add_argument('--max-jerk', type=float, default=2., help='continuous reference limit, rad/s^3')
    p.add_argument('--tracking-error', type=float, default=.05, help='measured vs previous sent target bound, rad')
    p.add_argument('--stale-after', type=float, default=.2, help='maximum state reception age, seconds')
    p.add_argument('--max-lateness', type=float, default=.05, help='maximum loop/send pause, seconds')
    p.add_argument('--kp', type=float, nargs=7, help='otherwise resolved from joint_impedance.yaml')
    p.add_argument('--kd', type=float, nargs=7, help='otherwise resolved from joint_impedance.yaml')
    p.add_argument('--filter-alpha', type=float, help='otherwise resolved from joint_impedance.yaml')
    p.add_argument('--finish', choices=['hold', 'terminate'], default='hold', help='normal completion; any runtime fault attempts termination')
    p.add_argument('--experiment-metadata', type=Path, help='optional JSON object: payload, mount, hand pose, robot/Desk versions')
    p.add_argument('--nuc-log', help='reference to separately collected NUC log; existence/content are NOT verified')
    p.add_argument('--output', type=Path, help='new directory; existing directories are never overwritten')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        fields = Plan.__dataclass_fields__
        settings = {name: getattr(args, name) for name in fields}
        settings['joints'], settings['tones'] = tuple(args.joints), tuple(args.tones)
        trajectory = Trajectory(Plan(**settings))
        for name in ('stale_after', 'tracking_error', 'max_lateness'):
            if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if any(not 1 <= port <= 65535 for port in (args.cmd_port, args.state_port)):
            raise ValueError('ports must be in 1..65535')
        if args.execute and (not args.host or not args.robot_id or args.q0_rad is not None):
            raise ValueError('--execute requires --host and --robot-id, and forbids --q0-rad')
        experiment = json.loads(args.experiment_metadata.read_text()) if args.experiment_metadata else {}
        if not isinstance(experiment, dict):
            raise ValueError('experiment metadata must be a JSON object')
        # Check JSON finiteness before any connection or motion.
        json.dumps(experiment, allow_nan=False)
        output = args.output or Path('recordings') / datetime.now(timezone.utc).strftime('joint-%Y%m%dT%H%M%S-%fZ')
        output.mkdir(parents=True, exist_ok=False)
        metadata = {
            'schema': 'fr3_joint_excitation/v1', 'status': 'initializing',
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'robot_id': args.robot_id, 'host': args.host, 'cmd_port': args.cmd_port,
            'state_port': args.state_port, 'plan': asdict(trajectory.plan),
            'experiment': experiment, 'nuc_log_reference': args.nuc_log,
            'nuc_log_verified': False, 'sysid_ready': False,
            'data_semantics': 'Client send attempts and latest PUB state sampled before each send; no daemon acknowledgement, no per-tick torque, no effective filtered target. Not a 1 kHz sysid recording.',
            'guard_settings': {name: getattr(args, name) for name in ('stale_after', 'tracking_error', 'max_lateness')},
            'commands_sent': 0, 'motion_command_attempted': False,
        }
        write_json(output / 'metadata.json', metadata)
        if not args.execute:
            q0 = vector(args.q0_rad if args.q0_rad is not None else PREVIEW_Q, 'preview anchor')
            metadata.update(status='preview', anchor_q_rad=q0.tolist(),
                            anchor_source='user' if args.q0_rad is not None else 'illustrative_only',
                            reference_peaks=trajectory.check(q0))
            write_preview(output, trajectory, q0)
            write_json(output / 'metadata.json', metadata)
            print(f'Offline preview: {output / "preview.svg"}\nReference: {output / "reference.csv"}')
            print(json.dumps(metadata['reference_peaks'], indent=2))
            return 0
        # Keep offline operation independent of pycapnp and of Robot creation.
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from fr3_stack.robot import Robot
        from fr3_stack.config import load_controller_config
        from fr3_stack.wire import flat_joint
        defaults = flat_joint(load_controller_config('joint_impedance'))
        gains = {
            'K_joint': vector(args.kp if args.kp is not None else defaults['K_joint'], 'K_joint').tolist(),
            'D_joint': vector(args.kd if args.kd is not None else defaults['D_joint'], 'D_joint').tolist(),
            'filter_alpha': args.filter_alpha if args.filter_alpha is not None else defaults['filter_alpha'],
            'use_friction': False,
        }
        if min(gains['K_joint']) <= 0 or min(gains['D_joint']) <= 0 or not 0 < gains['filter_alpha'] <= 1:
            raise ValueError('positive K/D and filter_alpha in (0, 1] required')
        metadata['commanded_controller_settings'] = gains
        metadata['controller_settings_acknowledged'] = False
        # SIGTERM follows the same partial-log / termination path as Ctrl-C.
        def interrupted(signum, frame):
            raise KeyboardInterrupt(f'signal {signum}')
        old_handler = signal.signal(signal.SIGTERM, interrupted)
        try:
            with Robot(args.host, args.cmd_port, args.state_port, command_send_timeout_ms=20) as robot:
                run_live(robot, trajectory, gains, output, metadata, args)
        finally:
            signal.signal(signal.SIGTERM, old_handler)
        print(f'Completed: {output}; end behavior: {args.finish}')
        if args.finish == 'hold':
            print('The NUC remains in joint impedance holding the anchor after this process exits.')
        else:
            print(f'Termination: {metadata.get("termination")}')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if 'metadata' in locals() and metadata.get('status') != 'aborted':
            metadata.update(status='failed', error=f'{type(exc).__name__}: {exc}')
            try:
                write_json(output / 'metadata.json', metadata)
            except OSError as save_error:
                print(f'Could not save failure metadata: {save_error}', file=sys.stderr)
        print(f'ERROR: {exc}', file=sys.stderr)
        if 'metadata' in locals() and metadata.get('termination'):
            print(f'Termination: {metadata["termination"]}; verify the robot locally.', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
