#!/usr/bin/env python3
"""Pair a frozen workstation SDK with a NUC image, without service activation."""

import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import importlib.util
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[1]
REMOTE_SCRIPT = Path(__file__).with_name("_nuc_sync_remote.py")
spec = importlib.util.spec_from_file_location("nuc_sync_remote", REMOTE_SCRIPT)
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)
CODE_PREFIXES = ("src/", "include/", "proto/", "third_party/", "fr3_stack/", "containers/")


def command(argv, cwd=None):
    result = subprocess.run(argv, capture_output=True, text=True, cwd=cwd)
    if result.returncode:
        raise RuntimeError(f"{argv[0]} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as output:
        json.dump(data, output, indent=2)
        output.write("\n")
        temporary = Path(output.name)
    os.replace(temporary, path)


def validate_target(target):
    if not isinstance(target, str) or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:\[\]-]*", target):
        raise RuntimeError("Set ssh_target to the real user@host or SSH alias; dummy IP is not usable")
    host = target.rsplit("@", 1)[-1].strip("[]")
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        if host.lower() in {"dummy", "dummy-ip", "example", "example.com", "nuc-placeholder"}:
            raise RuntimeError("Placeholder SSH target is not usable")
        return
    placeholders = [ipaddress.ip_network(n) for n in
                    ("192.0.2.0/24", "198.51.100.0/24", "203.0.113.0/24", "2001:db8::/32")]
    if address.is_loopback or address.is_unspecified or address.is_multicast or any(
        address.version == network.version and address in network for network in placeholders
    ):
        raise RuntimeError("Dummy/documentation/loopback SSH address is not a real NUC target")


def settings(path, args):
    if not path.is_file():
        raise RuntimeError("Missing local config; copy configs/nuc-sync.example.json to .setup/nuc-sync.json")
    config = json.loads(path.read_text())
    for key in ("ssh_target", "nuc_repo", "libfranka_version", "python"):
        value = getattr(args, key, None)
        if value is not None:
            config[key] = value
    if getattr(args, "jz", False):
        config["variant"] = "jz"
    config.setdefault("variant", "default")
    if config["variant"] not in {"default", "jz"}:
        raise RuntimeError("variant must be default or jz")
    config.setdefault("build_jobs", 4)
    if type(config["build_jobs"]) is not int or not 1 <= config["build_jobs"] <= 64:
        raise RuntimeError("build_jobs must be an integer between 1 and 64")
    if args.action != "build-local":
        validate_target(config.get("ssh_target"))
        if config.get("phase", "local_build") != "real_prepare":
            raise RuntimeError("Real NUC access is deferred; set phase=real_prepare when real preparation is requested")
    config.setdefault("docker_command", ["docker"])
    if config["docker_command"] not in (["docker"], ["sudo", "-n", "docker"]):
        raise RuntimeError("docker_command must be [docker] or [sudo, -n, docker]")
    config.setdefault("local_docker_command", config["docker_command"])
    if config["local_docker_command"] not in (["docker"], ["sudo", "-n", "docker"]):
        raise RuntimeError("local_docker_command must be [docker] or [sudo, -n, docker]")
    for key in ("nuc_repo", "release_root"):
        value = config.get(key)
        if not isinstance(value, str) or not (value.startswith("/") or value.startswith("~/")) or ".." in Path(value).parts:
            raise RuntimeError(f"{key} must be an absolute path or a ~/ path")
    config.setdefault("min_free_bytes", 10 * 1024**3)
    if not isinstance(config["min_free_bytes"], int) or config["min_free_bytes"] <= 0:
        raise RuntimeError("min_free_bytes must be a positive integer")
    config.setdefault("repository", "https://github.com/Robot-Dexterity-Lab/fr3_stack.git")
    if not isinstance(config["repository"], str) or not config["repository"].strip():
        raise RuntimeError("repository must be a nonempty source repository URL")
    return config


def resolve_ssh_target(target):
    # Preserve ordinary SSH host-key verification and the user's proxy/port settings.
    raw = command(["ssh", "-G", target])
    host = next((line.split(maxsplit=1)[1] for line in raw.splitlines() if line.startswith("hostname ")), None)
    validate_target(host)
    return host


def call_remote(action, config, log, archive=None):
    argv = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=2", config["ssh_target"],
            shlex.join(["python3", "-c", REMOTE_SCRIPT.read_text(), action, json.dumps(config)])]
    source = archive.open("rb") if archive else subprocess.DEVNULL
    try:
        with log.open("ab") as output, tempfile.TemporaryFile() as receipt_file:
            proc = subprocess.Popen(argv, stdin=source, stdout=receipt_file, stderr=subprocess.PIPE)
            # Remote stdout is a small JSON receipt; stream build stderr while retaining it.
            for line in iter(proc.stderr.readline, b""):
                output.write(line)
                output.flush()
                sys.stderr.buffer.write(line)
                sys.stderr.buffer.flush()
            code = proc.wait()
            receipt_file.seek(0)
            stdout = receipt_file.read()
    finally:
        if archive:
            source.close()
    try:
        receipt = json.loads(stdout)
    except (json.JSONDecodeError, UnicodeDecodeError):
        raise RuntimeError(f"NUC {action} failed (SSH exit {code}); see {log}") from None
    if code or "error" in receipt:
        raise RuntimeError(receipt.get("error", f"NUC {action} failed with exit {code}"))
    return receipt


def snapshot(repo, release_root, includes):
    base_commit = command(["git", "-C", str(repo), "rev-parse", "HEAD"])
    tracked = command(["git", "-C", str(repo), "ls-files", "-z"]).split("\0")
    paths = {p for p in tracked if p}
    extra = {str(remote.safe_relative(p)) for p in includes}
    untracked = command(["git", "-C", str(repo), "ls-files", "--others", "--exclude-standard", "-z"]).split("\0")
    missing = [p for p in untracked if (p.startswith(CODE_PREFIXES) or p in {"CMakeLists.txt", "pyproject.toml"}) and p not in extra]
    if missing:
        raise RuntimeError("New source files must be selected with --include: " + ", ".join(missing))
    for path in extra:
        if not (path.startswith(CODE_PREFIXES) or path.startswith(("scripts/", "tests/")) or path in {"sync.md", "configs/nuc-sync.example.json"}):
            raise RuntimeError("--include accepts source, tests, scripts or sync.md, not site configuration")
        if not (repo / path).is_file():
            raise RuntimeError(f"Missing explicitly included file: {path}")
    paths |= extra
    files = []
    # Make a private temporary snapshot first: hash the bytes actually copied, not a changing worktree.
    release_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".snapshot-", dir=release_root) as temp:
        staging = Path(temp) / "source"
        staging.mkdir()
        for name in sorted(paths):
            source = repo / remote.safe_relative(name)
            if not source.exists() and not source.is_symlink():
                continue  # Honor worktree deletions.
            target = staging / name
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_symlink():
                link = os.readlink(source)
                if not (source.parent / link).resolve().is_relative_to(repo.resolve()):
                    raise RuntimeError(f"Source symlink points outside repository: {name}")
                target.symlink_to(link)
                files.append({"path": name, "kind": "symlink", "target": link})
            else:
                content = source.read_bytes()
                executable = bool(source.stat().st_mode & 0o111)
                target.write_bytes(content)
                target.chmod(0o755 if executable else 0o644)
                files.append({"path": name, "kind": "file", "sha256": hashlib.sha256(content).hexdigest(), "executable": executable})
        for required in ("containers/Dockerfile", "proto/fr3.capnp", "pyproject.toml", "fr3_stack/__init__.py"):
            if not (staging / required).is_file():
                raise RuntimeError(f"Missing snapshot build/client input: {required}")
        identity = remote.tree_id(files)
        manifest = {"base_commit": base_commit, "tree_sha256": identity, "files": files,
                    "schema_sha256": hashlib.sha256((staging / "proto/fr3.capnp").read_bytes()).hexdigest(),
                    "worktree_changes": bool(command(["git", "-C", str(repo), "status", "--porcelain"]))}
        write_json(staging / remote.MANIFEST, manifest)
        destination = release_root / identity
        if destination.exists():
            # Reuse the recorded baseline for identical contents, even after an empty commit.
            manifest = remote.verify_tree(destination, identity)
        else:
            os.rename(staging, destination)
    archive = release_root / (identity + ".tar.gz")
    # Write atomically, including only manifested files (no ignored config/data).
    with tempfile.NamedTemporaryFile(dir=release_root, delete=False) as output:
        temporary = Path(output.name)
    try:
        with tarfile.open(temporary, "w:gz", dereference=False) as tar:
            for item in manifest["files"]:
                tar.add(destination / item["path"], arcname=item["path"], recursive=False)
            tar.add(destination / remote.MANIFEST, arcname=remote.MANIFEST, recursive=False)
        os.replace(temporary, archive)
    finally:
        temporary.unlink(missing_ok=True)
    return destination, archive, manifest


def client_info(python):
    code = "import fr3_stack,json; from fr3_stack.wire._schema import _locate_schema; print(json.dumps({'module':fr3_stack.__file__, 'schema':str(_locate_schema())}))"
    # A repository working directory would mask the new editable installation.
    return json.loads(command([python, "-c", code], cwd="/"))


def save_image(docker, image, path):
    temporary = path.with_suffix(".partial")
    try:
        with temporary.open("wb") as output:
            subprocess.run(docker + ["image", "save", image], stdout=output, check=True)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": str(path), "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


def install_client(python, release, manifest):
    # Retain the old editable checkout for rollback, and do not adjust dependency versions.
    previous = client_info(python)
    old_root = Path(previous["module"]).resolve().parents[1]
    if not (old_root / "pyproject.toml").is_file():
        raise RuntimeError("Expected an editable SDK checkout; record a rollback package before updating")
    command(["uv", "pip", "check", "--python", python])
    try:
        command(["uv", "pip", "install", "--python", python, "--no-deps", "-e", str(release)])
        command(["uv", "pip", "check", "--python", python])
        actual = client_info(python)
        if Path(actual["module"]).resolve() != (release / "fr3_stack/__init__.py").resolve():
            raise RuntimeError("Python imports a different SDK; inspect PYTHONPATH or other editable installs")
        if hashlib.sha256(Path(actual["schema"]).read_bytes()).hexdigest() != manifest["schema_sha256"]:
            raise RuntimeError("Installed client protocol does not match the NUC image")
    except Exception:
        command(["uv", "pip", "install", "--python", python, "--no-deps", "-e", str(old_root)])
        raise
    return {"python": python, "module": actual["module"], "schema_sha256": manifest["schema_sha256"],
            "rollback_source": str(old_root), "process_restart_required": True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "build", "build-local"))
    parser.add_argument("--config", type=Path, default=ROOT / ".setup/nuc-sync.json")
    parser.add_argument("--ssh-target")
    parser.add_argument("--nuc-repo")
    parser.add_argument("--libfranka-version")
    parser.add_argument("--python", help="Existing workstation deployment interpreter")
    parser.add_argument("--include", action="append", default=[], help="Explicit new source file to include")
    parser.add_argument("--jz", action="store_true", help="Use separate JZ source releases and fr3-stack-jz images")
    args = parser.parse_args(argv)
    report_dir = args.config.resolve().parent / "nuc-sync"
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    report = {"action": args.action, "time_utc": timestamp, "status": "started", "nuc_contacted": False,
              "image_built": False, "client_updated": False, "daemon_restarted": False,
              "hardware_validated": False}
    report_path = report_dir / (timestamp + ".json")
    report_dir.mkdir(parents=True, exist_ok=True)
    try:
        config = settings(args.config, args)
        report["variant"] = config["variant"]
        if args.action in {"build", "build-local"}:
            if args.action == "build-local" and not config.get("libfranka_version"):
                config["libfranka_version"] = "0.17.0"
            report["firmware_compatibility_verified"] = False
            if not re.fullmatch(r"\d+\.\d+\.\d+", config.get("libfranka_version") or ""):
                raise RuntimeError("Set an exact libfranka_version (x.y.z); verify firmware compatibility before activation")
            python = config.get("python")
            if not isinstance(python, str) or not Path(python).is_file():
                raise RuntimeError("Set python to the existing workstation deployment interpreter")
            command(["uv", "pip", "check", "--python", python])
            report["previous_client"] = client_info(python)
        log = report_dir / (timestamp + ".log")
        if args.action == "build-local":
            config["docker_command"] = config["local_docker_command"]
            config["nuc_repo"] = str(ROOT)
            config["release_root"] = str(report_dir / "releases")
            state = remote.inspect(config)
            report["build_host"] = state
        else:
            report["ssh_target"] = config["ssh_target"]
            report["resolved_host"] = resolve_ssh_target(config["ssh_target"])
            state = call_remote("inspect", config, log)
            report["nuc_contacted"] = True
            report["nuc"] = state
        schema = ROOT / "proto/fr3.capnp"
        report["workstation"] = {"source_commit": command(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
                                 "source_dirty": bool(command(["git", "-C", str(ROOT), "status", "--porcelain"])),
                                 "schema_sha256": hashlib.sha256(schema.read_bytes()).hexdigest()}
        if config.get("python"):
            report["workstation"]["client"] = client_info(config["python"])
        if args.action in {"build", "build-local"}:
            if state["build_blockers"]:
                raise RuntimeError("Build blocked while control services may be running; see nuc.build_blockers")
            includes = list(args.include)
            for name in ("scripts/sync-nuc", "scripts/sync_nuc.py", "scripts/_nuc_sync_remote.py", "sync.md", "configs/nuc-sync.example.json", "containers/compose.jz.yml"):
                if (ROOT / name).is_file():
                    includes.append(name)
            local_releases = remote.release_root({"release_root": str(report_dir / "releases"), "variant": config["variant"]})
            release, archive, manifest = snapshot(ROOT, local_releases, includes)
            config["tree_sha256"] = manifest["tree_sha256"]
            report["snapshot"] = {k: v for k, v in manifest.items() if k != "files"}
            if args.action == "build-local":
                print(f"Build log: {log}", flush=True)
                with log.open("w") as build_log, contextlib.redirect_stderr(build_log):
                    report["build"] = remote.build(config)
            else:
                report["upload"] = call_remote("upload", config, log, archive)
                report["build"] = call_remote("build", config, log)
            report["image_built"] = True
            if args.action == "build-local":
                report["image_archive"] = save_image(config["docker_command"], report["build"]["image"], report_dir / (remote.image_repository(config) + "-" + manifest["tree_sha256"][:24] + ".tar"))
            report["client"] = install_client(config["python"], release, manifest)
            report["client_updated"] = True
            report["status"] = "local_release_built_real_deployment_pending" if args.action == "build-local" else "paired_release_prepared_activation_pending"
        else:
            report["status"] = "checked"
        returncode = 0
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        report["status"] = "blocked_or_failed"
        report["error"] = str(exc)
        print(str(exc), file=sys.stderr)
        returncode = 2
    write_json(report_path, report)
    write_json(report_dir / "latest.json", report)
    print(f"Report: {report_path}")
    print(f"Status: {report['status']}")
    return returncode


if __name__ == "__main__":
    sys.exit(main())
