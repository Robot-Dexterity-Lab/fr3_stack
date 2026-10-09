"""Remote, foreground inspection and image preparation; never start a daemon."""

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import tempfile


MANIFEST = ".fr3-sync-manifest.json"
CONTROL_PROGRAMS = {"fr3-stack", "cartesian_test", "grav_comp_franka_style"}


def release_root(config):
    root = Path(config["release_root"]).expanduser()
    variant = config.get("variant", "default")
    if variant not in {"default", "jz"}:
        raise RuntimeError("Unknown release variant")
    return root / "jz" if variant == "jz" else root


def image_repository(config):
    return "fr3-stack-jz" if config.get("variant") == "jz" else "fr3-stack"


def run(argv):
    result = subprocess.run(argv, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"{argv[0]} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def tree_id(files):
    return hashlib.sha256(json.dumps(files, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def safe_relative(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or ".." in path.parts or str(path) != name:
        raise RuntimeError(f"Invalid snapshot path: {name!r}")
    return path


def verify_tree(root, expected):
    manifest = json.loads((root / MANIFEST).read_text())
    if tree_id(manifest["files"]) != expected or manifest["tree_sha256"] != expected:
        raise RuntimeError("Snapshot manifest identity mismatch")
    expected_names = {item["path"] for item in manifest["files"]} | {MANIFEST}
    actual_names = {str(p.relative_to(root)) for p in root.rglob("*")
                    if (p.is_file() or p.is_symlink()) and "__pycache__" not in p.parts}
    if actual_names != expected_names:
        raise RuntimeError("Snapshot contains missing or additional files")
    for item in manifest["files"]:
        path = root / safe_relative(item["path"])
        if item["kind"] == "symlink":
            if not path.is_symlink() or os.readlink(path) != item["target"]:
                raise RuntimeError(f"Snapshot symlink mismatch: {item['path']}")
            if not path.resolve().is_relative_to(root.resolve()):
                raise RuntimeError("Snapshot symlink points outside the release")
        elif item["kind"] == "file":
            if path.is_symlink() or not path.is_file():
                raise RuntimeError(f"Missing snapshot file: {item['path']}")
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise RuntimeError(f"Snapshot content mismatch: {item['path']}")
            if bool(path.stat().st_mode & 0o111) != item["executable"]:
                raise RuntimeError(f"Snapshot executable mode mismatch: {item['path']}")
        else:
            raise RuntimeError("Unknown snapshot entry kind")
    return manifest


def image_info(docker, name):
    result = subprocess.run(docker + ["image", "inspect", name], capture_output=True, text=True)
    if result.returncode:
        # Missing images are normal; daemon/permission failures are not.
        run(docker + ["info", "--format", "{{.ServerVersion}}"])
        return None
    obj = json.loads(result.stdout)[0]
    return {"id": obj["Id"], "labels": obj.get("Config", {}).get("Labels") or {}}


def inspect(config):
    repo = Path(config["nuc_repo"]).expanduser()
    source = {"path": str(repo), "commit": None, "dirty": None, "schema_sha256": None}
    if (repo / ".git").exists():
        source["commit"] = run(["git", "-C", str(repo), "rev-parse", "HEAD"])
        source["dirty"] = bool(run(["git", "-C", str(repo), "status", "--porcelain"]))
    schema = repo / "proto/fr3.capnp"
    if schema.is_file():
        source["schema_sha256"] = hashlib.sha256(schema.read_bytes()).hexdigest()
    docker = config["docker_command"]
    server = run(docker + ["info", "--format", "{{.ServerVersion}}"])
    ids = run(docker + ["ps", "-q", "--no-trunc"]).split()
    containers = []
    blockers = []
    if ids:
        for obj in json.loads(run(docker + ["inspect"] + ids)):
            image = image_info(docker, obj["Image"])
            labels = obj.get("Config", {}).get("Labels") or {}
            command = [obj.get("Path", "")] + (obj.get("Args") or [])
            service = labels.get("com.docker.compose.service")
            control = service in {"fr3-stack", "cart-test"} or any(
                program in " ".join(command) for program in CONTROL_PROGRAMS
            ) or "fr3-stack" in obj.get("Config", {}).get("Image", "")
            row = {"id": obj["Id"], "name": obj["Name"].lstrip("/"),
                   "image_id": obj["Image"], "image_labels": image["labels"],
                   "compose_project": labels.get("com.docker.compose.project"),
                   "compose_service": service, "may_control_robot": control}
            containers.append(row)
            if control:
                blockers.append("container:" + row["name"])
    # Check native daemons too, without recording arbitrary process arguments.
    processes = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            executable = Path((proc / "cmdline").read_bytes().split(b"\0", 1)[0].decode()).name
        except (OSError, UnicodeError):
            continue
        if executable in CONTROL_PROGRAMS:
            processes.append({"pid": int(proc.name), "program": executable})
            blockers.append("process:" + proc.name + ":" + executable)
    kernel = os.uname().release
    realtime = Path("/sys/kernel/realtime")
    rt = realtime.is_file() and realtime.read_text().strip() == "1"
    root = release_root(config)
    parent = root
    while not parent.exists():
        parent = parent.parent
    return {"hostname": os.uname().nodename, "architecture": os.uname().machine,
            "kernel": kernel, "realtime_kernel_verified": rt,
            "docker_server_version": server, "source": source,
            "running_containers": containers, "control_processes": processes,
            "build_blockers": blockers, "release_root": str(root),
            "free_bytes": shutil.disk_usage(parent).free}


def require_build_ready(config):
    state = inspect(config)
    if state["build_blockers"]:
        raise RuntimeError("Build blocked while a control service may be running: " +
                           ", ".join(state["build_blockers"]))
    if state["architecture"] != "x86_64":
        raise RuntimeError("The vendored Bota library requires an x86_64 build host")
    if state["free_bytes"] < config["min_free_bytes"]:
        raise RuntimeError("Insufficient free disk space for the configured build threshold")
    return state


def upload(config):
    require_build_ready(config)
    root = release_root(config)
    root.mkdir(parents=True, exist_ok=True)
    destination = root / config["tree_sha256"]
    if destination.exists():
        # Consume the input even on reuse; do not leave SSH writing into a closed pipe.
        while sys.stdin.buffer.read(1024 * 1024):
            pass
        verify_tree(destination, config["tree_sha256"])
        return {"release_dir": str(destination), "reused": True}
    with tempfile.TemporaryDirectory(prefix=".incoming-", dir=root) as temp:
        staging = Path(temp) / "source"
        staging.mkdir()
        names = set()
        with tarfile.open(fileobj=sys.stdin.buffer, mode="r|gz") as archive:
            for member in archive:
                relative = safe_relative(member.name)
                if member.name in names or not (member.isfile() or member.issym()):
                    raise RuntimeError("Duplicate or unsupported archive entry")
                names.add(member.name)
                target = staging / relative
                # Disallow extraction through a symlink created by an earlier entry.
                if any(p.is_symlink() for p in target.parents if p != staging):
                    raise RuntimeError("Archive path crosses a symlink")
                target.parent.mkdir(parents=True, exist_ok=True)
                if member.issym():
                    link = PurePosixPath(member.linkname)
                    if link.is_absolute() or not (target.parent / link).resolve().is_relative_to(staging.resolve()):
                        raise RuntimeError("Archive symlink escapes release directory")
                    target.symlink_to(member.linkname)
                else:
                    with target.open("xb") as output:
                        shutil.copyfileobj(archive.extractfile(member), output)
                    target.chmod(0o755 if member.mode & 0o111 else 0o644)
        verify_tree(staging, config["tree_sha256"])
        # os.rename fails for an existing nonempty directory; never replace a release.
        os.rename(staging, destination)
    return {"release_dir": str(destination), "reused": False}


def build(config):
    state = require_build_ready(config)
    root = release_root(config) / config["tree_sha256"]
    manifest = verify_tree(root, config["tree_sha256"])
    version = config["libfranka_version"]
    image = image_repository(config) + ":sync-" + config["tree_sha256"][:24] + "-lf-" + version
    expected_labels = {"org.opencontainers.image.revision": manifest["base_commit"],
                       "org.opencontainers.image.source": config["repository"],
                       "org.fr3-stack.source-tree.sha256": config["tree_sha256"],
                       "org.fr3-stack.schema.sha256": manifest["schema_sha256"],
                       "org.fr3-stack.libfranka.version": version}
    if config.get("variant") == "jz":
        expected_labels["org.fr3-stack.variant"] = "jz"
    previous = image_info(config["docker_command"], image)
    reuse = previous and all(previous["labels"].get(k) == v for k, v in expected_labels.items())
    if previous and not reuse:
        raise RuntimeError("Existing release image has conflicting labels; preserve it and investigate")
    if not reuse:
        argv = config["docker_command"] + ["build", "--progress=plain", "-f", str(root / "containers/Dockerfile"),
                "--build-arg", "LIBFRANKA_VERSION=" + version,
                "--build-arg", "BUILD_JOBS=" + str(config.get("build_jobs", 4))]
        for key, value in expected_labels.items():
            argv += ["--label", key + "=" + value]
        argv += ["-t", image, str(root)]
        # Build output streams over SSH; activation is handled separately.
        subprocess.run(argv, stdout=sys.stderr, stderr=sys.stderr, check=True)
    actual = image_info(config["docker_command"], image)
    if not actual or not all(actual["labels"].get(k) == v for k, v in expected_labels.items()):
        raise RuntimeError("Built image labels do not match the source snapshot")
    # Force dynamic symbol resolution, with no network/devices or controller args.
    help_output = run(config["docker_command"] + ["run", "--rm", "--network", "none",
                      "--cap-drop=ALL", "--env", "LD_BIND_NOW=1", image, "--help"])
    if "usage: fr3-stack" not in help_output:
        raise RuntimeError("Runtime dependency/help validation failed")
    return {"release_dir": str(root), "image": image, "image_id": actual["id"],
            "labels": actual["labels"], "reused": bool(reuse),
            "variant": config.get("variant", "default"),
            "compose_project": "fr3_stack_jz" if config.get("variant") == "jz" else None,
            "build_jobs": config.get("build_jobs", 4),
            "runtime_help_passed_network_disabled": True,
            "prebuild_state": state, "daemon_restarted": False}


def main():
    action, raw = sys.argv[1:]
    config = json.loads(raw)
    actions = {"inspect": inspect, "upload": upload, "build": build}
    try:
        print(json.dumps(actions[action](config)))
    except Exception as exc:
        print(json.dumps({"error": str(exc)}))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
