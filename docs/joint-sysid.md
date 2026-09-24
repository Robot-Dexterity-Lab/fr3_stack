# Joint excitation and recording

`examples/joint_sysid.py` generates joint-position references and can send them
to **one existing fr3-stack daemon in joint impedance mode**. It supports a
hold, sequential single-joint sine/chirp segments, and simultaneous multisine
excitation. It does not control XHand. Keep the hand in a fixed, recorded pose.

This is a trajectory/collection client, not a complete FrankaTwin identification
pipeline. The simulation repository stays independent and consumes files.

## Adopted identification route and existing interface

The goal is joint-space **simulation parameter identification**: replay the
same applied joint references through a matched simulation controller, compare
simulated and recorded joint motion, and fit supported dynamics parameters.
Recorded q is the comparison signal; it must not overwrite simulator state
each step. Inverse-dynamics regression is not the primary route in this plan.

`Robot.send_joint_impedance(q_target)` already provides continuous joint
targets in this stack. The missing real-side pieces are fit-ready 1 kHz
recording and export, not a new joint command interface. The reviewed
FrankaTwin revision `3c7b43ee95cc4631915191928dd2648dba8235c6` instead uses
Cartesian task impedance for SI; its `move_to_q()` is a blocking reset motion.
We reuse its replay/search method, not its Cartesian command path.

The independent `simtoolreal-fr3-xhand` repository owns the adopted design in
`docs/joint_space_sysid.md` and implementation checklist in
`docs/frankatwin_sim_todo.md`. Its validator, matched calibration controller,
optimizer, and profile loader remain planned. It must consume portable files
without this stack's imports, source tree, daemon, or network connection.

The current real control law is target EMA followed by
`K*(filtered_target-q)-D*dq+Coriolis+joint_limit_repulsion`, with optional
friction compensation and final stack torque slew limiting. Filter state
resets to measured q. It does not use desired-velocity feedforward. Match the
actual run settings, torque/gravity convention, and downstream processing in
simulation before allowing an optimizer to change physical parameters.

## Preview first

Run commands from the repository root. `uv run` installs the project's declared
Python dependencies into its environment; an existing installation can instead
use `python examples/joint_sysid.py ...`.

```bash
uv run python examples/joint_sysid.py --dry-run --output /tmp/joint-preview
```

The default is a **J1 sine, 1 degree peak offset, 0.1 Hz, 20 seconds**, including
5-second fade-in and fade-out, with 2 seconds of holding before and after.
These are initial exploration settings, not validated parameters for your robot.
The default command rate is 60 Hz. Preview mode never connects to a robot,
including when `--host` is supplied. It only needs NumPy when run directly.

Open `preview.svg` to inspect offset, velocity, acceleration, and jerk.
`reference.csv` contains absolute joint targets and continuous-reference
derivatives in SI units. The default preview anchor is **illustrative**; use
`--q0-rad` followed by seven measured joint angles to preview your actual pose.
Execution never moves to this illustrative pose or accepts a preview anchor.

Each output directory must be new; omit `--output` for a timestamped directory
under `recordings/`. Keep generated recordings out of commits.

## First real run

Confirm the installed payload in Desk and inspect the full arm/hand motion
volume at the intended starting pose. The script checks joint ranges, but has
no robot/scene collision model. Complete the stack's ordinary hardware setup
and [connection procedure](quickstart.md). The NUC must already publish healthy,
stationary `joint_impedance` state (launch mode `joint`). Stop other command
producers; there is no exclusive ownership lease.

On the workstation, replace `NUC_HOST` with the NUC's hostname or address:

```bash
# Observe holding behavior and logging before introducing an oscillation.
uv run python examples/joint_sysid.py --execute --host NUC_HOST \
  --robot-id left --mode hold --seconds 20

# The default one-joint, low-frequency sine; anchor is the LIVE joint pose.
uv run python examples/joint_sysid.py --execute --host NUC_HOST \
  --robot-id left --mode sine --joints 1 --amplitude-deg 1 --f0 0.1
```

The client rejects missing/stale/invalid state, daemon errors, a different
controller, or initial joint speed above 0.02 rad/s. It checks the anchor again
after saving its preview and refuses to start if the arm has moved more than
0.002 rad while preparing. It does not reset or home the robot.

It explicitly sends K/D and target-filter settings resolved from the installed
`joint_impedance.yaml` on every command; `--kp`, `--kd`, and `--filter-alpha`
override them. Defaults are therefore **configuration dependent**. Friction
compensation is explicitly disabled for these experiments. Review the selected
gains before execution; no gain ramp is implemented. The script neither reads
nor changes Desk's payload or collision settings. A client send is not an
acknowledgement that the NUC applied those settings.

Normal completion returns the reference smoothly to the starting joint pose,
holds, and leaves the NUC holding that pose in joint impedance after Python
exits. `--finish terminate` instead requests daemon termination at the end.
Ctrl-C, SIGTERM, send failures, logging failures during streaming, stale
feedback, excessive measured speed/tracking error, or a missed deadline abort
the run and attempt termination **if a motion command was attempted**. A fault
does not command a return to the starting pose.

Termination goes over the same network and is recorded as unconfirmed; it is
not a hardware emergency stop. A disconnected/killed workstation cannot
guarantee a stop, and this client adds no NUC watchdog. Keep the existing local
stop procedure available. Preflight failures send no control commands.

## Chirp and multisine

These examples generate **offline previews**. Select amplitudes and frequency
bands from the actual payload, available space, and observed tracking/torque
response before adding `--execute --host NUC_HOST --robot-id left`.

```bash
# J1 through J7 in sequence, each sweeping 0.1 to 0.5 Hz for 40 seconds.
uv run python examples/joint_sysid.py --mode chirp --joints 1 2 3 4 5 6 7 \
  --seconds 40 --ramp 5 --amplitude-deg 1 --f0 0.1 --f1 0.5

# Simultaneous joint motion; seed fixes independent phases for reproducibility.
uv run python examples/joint_sysid.py --mode multisine --joints 1 2 3 4 5 6 7 \
  --seconds 40 --amplitude-deg 1 --tones 0.1 0.23 0.41 --seed 0

# Reserve this complete run for validation; do not use it to tune the fit.
uv run python examples/joint_sysid.py --mode multisine --joints 1 2 3 4 5 6 7 \
  --seconds 40 --amplitude-deg 1 --tones 0.13 0.29 0.47 --seed 101
```

For sine/chirp, `--seconds` is per selected joint, with `--gap` holding between
segments. For multisine it is the complete excitation duration. `--hold` adds
holding at both ends. Seventh-order envelopes make offset and its first three
derivatives zero at segment boundaries. Multisine tone amplitudes are weighted
by inverse frequency and normalized so their **sum**, not each tone, is bounded
by `--amplitude-deg`. Chirp uses constant amplitude within its envelope.

Position bounds conservatively include the full amplitude on every selected
joint and exclude the stack's outer 10% joint-limit repulsion regions.
Analytic reference derivatives, including the envelopes, are sampled at 1 kHz
for preflight checks. Default per-joint bounds are 0.1 rad/s, 0.3 rad/s^2, and
2 rad/s^3; they are configurable experiment limits, not certified hardware
limits. They do not predict torque, collisions, or the derivatives of the
sample-and-hold commands. The daemon still filters those commands at 1 kHz.

During streaming, the default state age bound is 0.2 s, tracking-error bound is
0.05 rad relative to the previous sent target, and loop/send pause bound is
0.05 s. Commands are evaluated at actual wall time. Expired scheduling slots
are skipped, and large pauses abort instead of causing a burst of old commands.
Terminal settling also requires measured speed at or below 0.02 rad/s.

## Recording contract and remaining 1 kHz work

| File | Contents |
| --- | --- |
| `preview.svg` | Continuous reference curves |
| `reference.csv` | Planned absolute targets and derivatives at the requested command rate |
| `commands_and_state.csv` | Successful client send attempts, send-start/return times, and latest measured q/dq sampled before each send |
| `metadata.json` | Schema, script hash, robot ID, anchor, waveform/seed, explicit commanded gains/filter, bounds, run outcome, and supplied experiment metadata |

`commands_and_state.csv` samples the approximately 200 Hz PUB stream at the
client's command rate. Repeated state samples are possible. Client monotonic
times and daemon timestamps are separate clocks; no clock synchronization is
claimed. Failed sends might still have reached the daemon and are recorded as
an aborted run, not assumed absent from the real motion. CSV output contains
no per-tick torque or effective filtered target.

**These files are not a 1 kHz system-identification dataset.** Metadata always
sets `sysid_ready=false` and `nuc_log_verified=false`. `--nuc-log` only records
the path or identifier of a separately collected NUC log; it does not start,
retrieve, synchronize, or validate that log. This repository implements
CSV v2 joint-target logging, writer-error handling, and independent
`Robot.start_recording` / `stop_recording` / `recording_status` plus a
`fr3-recording` CLI on port 5557. See [Recording](recording.md) for
the acknowledged recording lifecycle and persistent container output mount.
These APIs require the updated NUC daemon and Python client; the excitation
script still only records the `--nuc-log` reference.
Full daemon build, hardware acceptance and portable export are still pending.
These changes are not installed on the robot by this script. Do not fit
millisecond delay from this client's command/PUB log alone.

Supply hardware provenance without changing controller behavior:

```bash
uv run python examples/joint_sysid.py --execute --host NUC_HOST --robot-id left \
  --experiment-metadata /path/to/experiment.json \
  --nuc-log /path/on/nuc/to/independent-1khz.csv
```

The optional JSON object can contain robot/Desk versions, mount order, measured
payload mass, flange-frame center of mass and inertia, fixed hand joint pose,
and software revisions. Unknown values should remain null, not fabricated or
copied from the illustrative sim asset. Use distinct robot IDs and output
directories for the two arms, and validate each independently.

## Required handoff to simulation

The planned portable bundle is `joint_sysid_recording/v1` with `metadata.json`,
`samples.csv`, and a conversion/validation report. This is a draft contract, not
an implemented output option in this script. The excitation script still
emits `fr3_joint_excitation/v1` diagnostics with `sysid_ready=false`.

| Producer requirement | Status and acceptance evidence |
| --- | --- |
| Writer checks for write, flush, and close failures | Implemented/tested; failed I/O and drops invalidate completion; NUC acceptance pending |
| Per-tick active pre-EMA and effective post-EMA joint targets | Implemented/tested in CSV v2, with applied K/D/alpha/friction flag and reset count; NUC acceptance pending |
| State/timing/torque semantics | Shared capture tests pass; CSV v2 includes robot state time and actual callback period alongside the existing host-end timestamp; full dispatch/hardware verification pending |
| Applied configuration and initialization | Verified K/D/alpha, compensations, limits, reset state and warm-up; not only client-requested values |
| Payload and frame provenance | Physical assembly and configured loads recorded separately, with source, uncertainty, hand pose, and asset/frame definitions |
| Portable exporter | Explicit joint-name/unit mapping, original hashes, versioned metadata, and rejection of invalid/gapped/aborted runs without sibling imports |

The logger's callback-end `t_s` must not be silently relabeled as a
state timestamp. CSV v2 adds `robot_time_s` and `control_period_s` explicitly.
Host send and NUC application use different clocks. Per-tick
active targets are needed to replay actual holds; delay fitting must not absorb
unknown timestamp alignment or count upstream command delay twice. Preserve raw
files and do not interpolate missing ticks into an apparently complete run.

Physical payload is unresolved for the reported FR3 -> mount -> XHand assembly.
The calibrated URDF's XHand mass sum (`1.10146642 kg`) excludes a separately
modeled mount inertia and is not a measurement of the complete physical load.
Record physical mass/CoM/inertia and configured Desk values independently; a
configured zero additional load does not establish that no hand is installed.

## Delivery order

1. Fix logging and freeze the portable contract; develop synthetic validation
   and controller fixtures while hardware provenance is being established.
2. Accept real hold/small-motion recordings and single-environment nominal
   replay with fixed hand posture and documented payload assumptions.
3. Collect informative chirp/multisine runs and reserve whole independent runs;
   freeze known gains, filtering, compensation, and timing during each fit.
4. Fit only identifiable parameter groups with fixed-scale q/dq error, finite
   justified bounds, and a baseline; inspect repeated seeds and bound hits.
5. Validate on untouched recordings against predeclared task thresholds,
   export a versioned profile, and reproduce its scores from saved artifacts.
6. Evaluate training transfer separately: current sim implicit PD at 120 Hz
   physics / 60 Hz policy is not proven equivalent to real 1 kHz impedance.
   A 3 ms actuator delay is not three steps at either training rate.

No formal fitting should start before complete recordings and controller
parity are established. The root `TODO_FRANKATWIN.md` tracks real-side tasks
and cross-repository acceptance; the sim design owns the detailed loss,
parameter/profile contract, and training-transfer gates.
