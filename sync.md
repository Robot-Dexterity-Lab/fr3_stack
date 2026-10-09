# Work PC and NUC synchronization

Updating the work-PC Python SDK does not update the NUC's compiled C++ daemon.
This guide defines source selection, image preparation, verification and recovery.
The work-PC agent owns the complete workflow and runs ordinary NUC commands over
SSH. The NUC does not need Codex or a resident agent.

Read the update rules in [AGENTS.md](AGENTS.md) before making deployment changes.
Build preparation and physical-robot activation are separate tasks.

## Configuration and source ownership

The work PC maintains the source and selects the release. Keep real SSH targets,
site settings and interpreter paths in the ignored `.setup/nuc-sync.json` file.
Never store passwords there. Copy [the example](configs/nuc-sync.example.json)
for a new installation and set `python` to the existing deployment interpreter.
The work PC needs Git, Python 3, Docker and the `uv` CLI; real preparation also
needs SSH access to a NUC with Python 3 and Docker.

- `phase=local_build`: prepare and export an image on the work PC without SSH.
- `phase=real_prepare`: inspect and prepare a real NUC after the user has arranged
  that preparation. This phase does not authorize controller activation.
- `docker_command` and `local_docker_command`: use `["docker"]`, or
  `["sudo", "-n", "docker"]` when the appropriate terminal is already authenticated.
- `build_jobs`: C++ build parallelism; default 4, typically 2 on a smaller NUC.
- `variant=jz`: prepare the separate JZ image and source namespace.
- `nuc_sdk_python`: optional existing NUC diagnostic interpreter. The work-PC
  agent updates this SDK over SSH after the build; the current build command does
  not install it automatically.

Treat the configured phase and actual receipts as authoritative. A documentation
example, checkout commit or package version is not evidence of the deployed
binary. Firmware compatibility and physical-robot validation require separate
site checks; the default libfranka version is only a build default.

## Repeatable preparation

Run these commands on the work PC, from the stack checkout:

```bash
# Local preparation: no SSH or controller startup.
bash scripts/sync-nuc build-local

# Real NUC preparation: requires a configured real_prepare phase.
bash scripts/sync-nuc check --jz
bash scripts/sync-nuc build --jz
```

The configured model checkout may also provide the local entry point:

```bash
bash .deploy/sync_nuc.sh check --jz
bash .deploy/sync_nuc.sh build --jz
```

An update follows this sequence:

1. Run relevant work-PC tests. Inspect SSH access, Docker permissions, architecture,
   kernel, free disk space, running containers and native control processes.
   Preparation is blocked while a recognized controller is running.
2. Freeze the current Git-tracked work-PC files, including reviewed uncommitted
   changes and deletions. New source files require explicit `--include path`
   selection. Built-in synchronization scripts and this guide are included.
   Ignored site settings, passwords, calibration overrides and recordings stay
   outside the source archive.
3. Hash the frozen source and schema. Upload an independent release directory,
   validate its manifest and build the image over SSH. The local action instead
   builds and exports an image archive on the work PC.
4. Build and run the C++ tests. Validate runtime dynamic dependencies using only
   `--help` with networking disabled and capabilities dropped. Reuse an existing
   image only when its expected labels match; conflicting labels or damaged
   source snapshots block preparation.
5. Install the work-PC SDK from the same frozen source. Check its actual import
   path, schema hash and dependencies. A failed build leaves the SDK unchanged;
   failed SDK validation restores its previous editable source. Already-running
   client processes need restarting after a successful code update.
6. If configured, use SSH to install the NUC diagnostic SDK from that same remote
   release, check its imports and dependencies, and compare its schema hash.
   Keep model inference and GPU dependencies on the work PC.
7. Prepare the exact image and source references in the ignored NUC launch
   configuration. Validate Compose `config` and preserve the existing external
   site data. Record previous SDK sources and launch references before changing
   them. These additional NUC SDK/launch steps are agent-orchestrated; they are
   not automatic in `build`.
8. Save actual source/schema hashes, SDK paths, image IDs, validation results and
   recovery references. Restore changed preparation references on a detected
   failure where reachable. If recovery is blocked, record the partial state
   instead of reporting paired success.

The commands never stop/start controllers or send robot commands. An active
controller remains a blocker until the operator completes its existing task.
SSH uses existing keys and normal host-key verification; dummy/documentation IPs
and unconfigured real phases are rejected before contacting a NUC.

Receipts and logs live in `.setup/nuc-sync/`, with a separate timestamped JSON
record for each invocation. `latest.json` is the latest invocation, which may be
an inspection rather than the latest successful build. Retain the successful
build receipt explicitly for later verification and recovery.

## Release identity and JZ isolation

A frozen source content hash distinguishes uncommitted changes at one Git commit.
The image revision label records the Git baseline; the actual source identity is
`org.fr3-stack.source-tree.sha256`. Protocol identity is
`org.fr3-stack.schema.sha256`. Record the exact libfranka version and image ID.

The JZ variant uses:

- Source releases under `<release_root>/jz/<full source SHA256>/`.
- Images named `fr3-stack-jz:sync-<first 24 source hash characters>-lf-<version>`.
- Image label `org.fr3-stack.variant=jz`.
- Compose project `fr3_stack_jz`, selected by `fr3-stack --jz`.

The JZ launcher applies `containers/compose.jz.yml`. It requires
`FR3_JZ_IMAGE` to name the exact prepared image, `FR3_JZ_CALIB_DIR` to point to an
external calibration directory, and `FR3_JZ_RECORDINGS_DIR` to point to an
external recordings directory. Review `config` before separately arranged
activation:

```bash
./fr3-stack --jz config
```

The original and JZ services share the same robot and ports and cannot control
it simultaneously. A separate Compose project preserves the original project's
containers; it does not enable concurrent robot control. Preserve the previous
image, source and launch configuration for recovery.

## NUC user-directory installation

Clone the same repository and Git baseline into the NUC user's `~/fr3_stack_jz`.
A Git clone alone does not contain the work PC's uncommitted changes. During
initial installation, apply the verified selected source to the new, untouched
checkout and record its file manifest. Subsequent deployment normally uploads
independent frozen releases.

Update a NUC development checkout only after comparing it with its last installed
manifest. Preserve unexpected edits and return them to the work PC for review.
Do not force-pull, reset or overwrite a dirty checkout to make the update pass.
For documentation-only changes, synchronize the affected guides and their
checkout manifest while retaining the verified SDK/image release.

The work-PC configuration points to the NUC user's SSH target, checkout and
release directory. `.setup/site-role.json` with `role=nuc` identifies an execution
site, not a resident agent. The NUC's local configuration can use
`phase=local_build`, existing Docker access and its diagnostic interpreter for
commands executed there through SSH. A NUC-only build does not update the work-PC
client or establish a paired release.

When installed, the NUC's ignored `.setup/activate.sh` activates its SDK
environment, loads site settings and selects the verified release directory:

```bash
cd ~/fr3_stack_jz
source .setup/activate.sh
bash "$FR3_RELEASE_DIR/fr3-stack" --jz config
```

Keep calibration and recordings outside the checkout. Copy existing calibration
only with content/hash verification; calibration changes are a separate task.
A normal `git pull` does not replace paired SDK/image preparation.

## Which changes require an update

| Change | Work PC | NUC |
| --- | --- | --- |
| Python SDK implementation | Install/test the selected SDK | Refresh a configured diagnostic SDK; assess interface compatibility |
| C++ controller or daemon | Test client compatibility | Build a matched image; activate only in a separate hardware task |
| Cap'n Proto schema or wire semantics | Update/test serialization | Regenerate and compile the same schema |
| Dockerfile, C++ dependencies, libfranka | Record compatibility | Rebuild; verify firmware support before activation |
| Python dependencies or package layout | Validate the existing deployment environment | Update diagnostic/build dependencies only when needed |
| Model/checkpoint/contract or peripheral adapters | Run model and deployment checks | No daemon rebuild for model-only changes |
| Documentation and agent rules | Update guides | Synchronize guides; retain the verified runtime release |
| Addresses, ports, gains, calibration, recordings | Preserve actual site settings | Keep site data outside source releases |

Both packages reporting `0.1.0` does not establish source compatibility. Prefer
one verified source selection for changes affecting the protocol or both ends.
Describe compatibility explicitly when only one component changes.

## Release records

Record at least the following in the ignored setup directory:

| Evidence | What to verify |
| --- | --- |
| Source | Full Git baseline and frozen source content hash |
| SDKs | Actual `fr3_stack.__file__` paths and schema hashes |
| Prepared image | Exact image ID, source/schema labels and libfranka version |
| Running service, if activated | Actual container image ID, project, arguments and mounts |
| Site | Kernel, robot firmware, addresses, ports and service management |
| Persistent data | Actual calibration/recording paths and relevant calibration hashes |
| Validation/recovery | Time, results, previous SDK sources, image and launch references |

A checkout's `git rev-parse HEAD` cannot identify the running binary. An old image
without provenance labels remains unverified until its original build evidence
is checked. The model project's historical `fr3_stack_reference` log field is
not a probe of the installed SDK or running NUC image.

## Separately arranged activation and verification

Complete the current control task and the site's maintenance procedure before
switching a daemon. Stopping a Python policy does not stop the NUC controller:
it can retain its last target. Building on the NUC consumes CPU/memory; perform
it after control has ended, or build elsewhere and transfer the exact image.

Before activation, verify firmware/libfranka compatibility, the old service's
stop/recovery method, reviewed controller mode, addresses, ports, sensor settings,
and external calibration/recording mounts. The model deployment requires a
reviewed joint-impedance controller; changing controller mode is a hardware task.

For an explicitly arranged activation, use the reviewed Compose project,
immutable release and exact image with `up --no-build --pull never fr3-stack` in
the foreground. Stop the previous service using its existing management method
first, and confirm that no other controller owns the robot. Preparation itself
never runs this activation command.

Validate in layers:

1. Work-PC SDK/schema tests and localhost FakeDaemon communication.
2. The activated NUC container's actual image ID/labels, kernel and startup logs.
3. The model project's read-only `model/check_connection.py REAL_NUC_IP` check
   for seven-joint state, controller status and advancing feedback timestamps.
4. Offline checkpoint/config/contract inference and replay, then the full
   perception chain in shadow mode.
5. Separately supervised execution after reviewing site parameters, initial pose
   and the complete control chain. Initialize from current measured state;
   do not replay the last target from before the update.

Only actual image and feedback verification can establish an activated NUC
release. Build success, matching schema files and localhost tests alone cannot.

## Recovery and maintenance

For preparation failures, keep logs and restore changed SDK/launch references
from the last verified receipt where reachable. The SDK installer automatically
restores its old source on local validation failure. If the NUC cannot be reached,
report the unconfirmed remote state and preserve the recovery references.

For an activated release that fails validation, follow the site's stop procedure
and restore the previous actual image, source, SDK and original launch settings.
Verify the restored image and read-only feedback before continuing. Keep old
images, configurations, calibration and recordings until the maintenance window
has ended. Track gain, filter, firmware and calibration changes separately so a
code rollback does not leave incompatible site settings.

Update this guide, the agent rules and [CHANGELOG.md](CHANGELOG.md) when protocol,
daemon, container or deployment behavior changes. Receipts record actual runs;
this guide describes the reusable workflow. Current tooling records identity
through SSH and image labels; it does not implement a daemon protocol version
handshake. Document any future handshake's compatibility contract separately.
