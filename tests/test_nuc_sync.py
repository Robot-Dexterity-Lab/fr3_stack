"""Hardware-free deployment guards, source integrity, and client rollback."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from unittest.mock import Mock

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("sync_nuc", ROOT / "scripts/sync_nuc.py")
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)
remote = sync.remote


@pytest.mark.parametrize("target", [None, "192.0.2.8", "user@198.51.100.2", "203.0.113.3", "127.0.0.1", "0.0.0.0", "[2001:db8::1]", "-oProxyCommand=whoami", "user@host;touch x"])
def test_invalid_target_is_rejected_before_ssh(target):
    with pytest.raises(RuntimeError):
        sync.validate_target(target)


def test_local_build_does_not_need_real_address(tmp_path):
    path = tmp_path / "config.json"
    path.write_text((ROOT / "configs/nuc-sync.example.json").read_text())
    config = sync.settings(path, argparse.Namespace(action="build-local"))
    assert config["ssh_target"] is None
    with pytest.raises(RuntimeError, match="ssh_target"):
        sync.settings(path, argparse.Namespace(action="check"))


def test_jz_variant_uses_separate_releases_and_images(tmp_path):
    path = tmp_path / "config.json"
    path.write_text((ROOT / "configs/nuc-sync.example.json").read_text())
    config = sync.settings(path, argparse.Namespace(action="build-local", jz=True))
    assert config["variant"] == "jz"
    assert remote.release_root(config) == Path(config["release_root"]).expanduser() / "jz"
    assert remote.image_repository(config) == "fr3-stack-jz"
    default = sync.settings(path, argparse.Namespace(action="build-local"))
    assert remote.release_root(default) == Path(default["release_root"]).expanduser()
    assert remote.image_repository(default) == "fr3-stack"


@pytest.mark.parametrize("repository", [None, "https://github.com/example/fr3-stack.git"])
def test_source_repository_defaults_to_upstream_and_preserves_fork(tmp_path, repository):
    path = tmp_path / "config.json"
    value = json.loads((ROOT / "configs/nuc-sync.example.json").read_text())
    value.pop("repository", None)
    if repository is not None:
        value["repository"] = repository
    path.write_text(json.dumps(value))
    config = sync.settings(path, argparse.Namespace(action="build-local"))
    assert config["repository"] == (repository or "https://github.com/Robot-Dexterity-Lab/fr3_stack.git")


@pytest.mark.parametrize("value", [0, -1, True, "2", 65])
def test_invalid_build_parallelism_is_rejected(tmp_path, value):
    path = tmp_path / "config.json"
    config = json.loads((ROOT / "configs/nuc-sync.example.json").read_text())
    config["build_jobs"] = value
    path.write_text(json.dumps(config))
    with pytest.raises(RuntimeError, match="build_jobs"):
        sync.settings(path, argparse.Namespace(action="build-local"))


@pytest.fixture
def tiny_repo(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    files = {"containers/Dockerfile": "FROM ubuntu:24.04\n", "proto/fr3.capnp": "@0xabcd;\n",
             "pyproject.toml": "[project]\nname = 'fr3-stack'\n", "fr3_stack/__init__.py": "# client\n",
             ".gitignore": ".setup/\nfr3.yaml\n__pycache__/\n"}
    for name, content in files.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    for args in (["init", "-q"], ["add", "."], ["-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture"]):
        subprocess.run(["git", "-C", str(repo)] + list(args), check=True)
    (repo / "fr3.yaml").write_text("private site config")
    (repo / ".setup").mkdir()
    (repo / ".setup/password").write_text("not exported")
    return repo


def test_snapshot_includes_dirty_source_but_not_site_files(tiny_repo, tmp_path):
    client = tiny_repo / "fr3_stack/__init__.py"
    client.write_text("# uncommitted new client\n")
    release, archive, manifest = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    assert (release / "fr3_stack/__init__.py").read_text() == client.read_text()
    assert not (release / "fr3.yaml").exists()
    assert not (release / ".setup").exists()
    assert manifest["worktree_changes"]
    assert remote.verify_tree(release, manifest["tree_sha256"]) == manifest
    again, _, second = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    assert again == release
    assert second["tree_sha256"] == manifest["tree_sha256"]
    assert archive.is_file()


def test_identical_contents_keep_the_reused_manifest_baseline(tiny_repo, tmp_path):
    release, _, first = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    subprocess.run(["git", "-C", str(tiny_repo), "-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "--allow-empty", "-qm", "new baseline"], check=True)
    again, _, second = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    assert again == release
    assert second == first


def test_unknown_new_source_requires_explicit_inclusion(tiny_repo, tmp_path):
    (tiny_repo / "fr3_stack/new.py").write_text("new implementation\n")
    with pytest.raises(RuntimeError, match="--include"):
        sync.snapshot(tiny_repo, tmp_path / "releases", [])
    release, _, _ = sync.snapshot(tiny_repo, tmp_path / "releases", ["fr3_stack/new.py"])
    assert (release / "fr3_stack/new.py").is_file()


def test_snapshot_rejects_outside_symlink(tiny_repo, tmp_path):
    (tiny_repo / "fr3_stack/external.py").symlink_to(tmp_path / "secret")
    (tmp_path / "secret").write_text("outside")
    with pytest.raises(RuntimeError, match="outside"):
        sync.snapshot(tiny_repo, tmp_path / "releases", ["fr3_stack/external.py"])


def test_modified_release_and_extra_files_are_rejected(tiny_repo, tmp_path):
    release, _, manifest = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    extra = release / "unmanifested-input"
    extra.write_text("extra")
    with pytest.raises(RuntimeError, match="additional"):
        remote.verify_tree(release, manifest["tree_sha256"])
    extra.unlink()
    (release / "proto/fr3.capnp").write_text("corrupted")
    with pytest.raises(RuntimeError, match="content mismatch"):
        remote.verify_tree(release, manifest["tree_sha256"])


def test_cache_creation_does_not_change_source_identity(tiny_repo, tmp_path):
    release, _, manifest = sync.snapshot(tiny_repo, tmp_path / "releases", [])
    cache = release / "fr3_stack/__pycache__"
    cache.mkdir()
    (cache / "generated.pyc").write_bytes(b"bytecode")
    assert remote.verify_tree(release, manifest["tree_sha256"])["tree_sha256"] == manifest["tree_sha256"]


@pytest.mark.parametrize("variant", ["default", "jz"])
def test_running_controller_blocks_build_before_docker_build(monkeypatch, variant):
    monkeypatch.setattr(remote, "inspect", lambda _: {"build_blockers": ["container:active"]})
    launch = Mock()
    monkeypatch.setattr(remote.subprocess, "run", launch)
    with pytest.raises(RuntimeError, match="running"):
        remote.build({"variant": variant})
    launch.assert_not_called()


def test_reused_image_is_checked_with_no_network_or_controller(monkeypatch, tmp_path):
    config = {"release_root": str(tmp_path), "tree_sha256": "a" * 64,
              "libfranka_version": "0.17.0", "repository": "repo", "docker_command": ["docker"]}
    labels = {"org.opencontainers.image.revision": "baseline", "org.opencontainers.image.source": "repo",
              "org.fr3-stack.source-tree.sha256": "a" * 64, "org.fr3-stack.schema.sha256": "schema",
              "org.fr3-stack.libfranka.version": "0.17.0"}
    monkeypatch.setattr(remote, "require_build_ready", lambda _: {})
    monkeypatch.setattr(remote, "verify_tree", lambda *_: {"base_commit": "baseline", "schema_sha256": "schema"})
    monkeypatch.setattr(remote, "image_info", lambda *_: {"id": "image-id", "labels": labels})
    invoke = Mock(return_value="usage: fr3-stack --robot <ip>")
    monkeypatch.setattr(remote, "run", invoke)
    result = remote.build(config)
    assert result["reused"]
    assert result["runtime_help_passed_network_disabled"]
    argv = invoke.call_args.args[0]
    assert argv[argv.index("--network") + 1] == "none"
    assert "--cap-drop=ALL" in argv and "LD_BIND_NOW=1" in argv
    assert argv[-1] == "--help" and "--robot" not in argv


def test_jz_build_labels_and_runtime_probe_are_isolated(monkeypatch, tmp_path):
    config = {"release_root": str(tmp_path), "tree_sha256": "a" * 64, "variant": "jz",
              "build_jobs": 2, "libfranka_version": "0.17.0", "repository": "repo",
              "docker_command": ["docker"]}
    monkeypatch.setattr(remote, "require_build_ready", lambda _: {})
    verify = Mock(return_value={"base_commit": "baseline", "schema_sha256": "schema"})
    monkeypatch.setattr(remote, "verify_tree", verify)
    invocations = []
    labels = {}
    def compile_image(argv, **_kwargs):
        invocations.append(argv)
        for i, item in enumerate(argv):
            if item == "--label":
                key, value = argv[i + 1].split("=", 1)
                labels[key] = value
    monkeypatch.setattr(remote.subprocess, "run", compile_image)
    metadata = Mock(side_effect=[None, {"id": "jz-image", "labels": labels}])
    monkeypatch.setattr(remote, "image_info", metadata)
    help_probe = Mock(return_value="usage: fr3-stack --robot <ip>")
    monkeypatch.setattr(remote, "run", help_probe)
    result = remote.build(config)
    image = "fr3-stack-jz:sync-" + "a" * 24 + "-lf-0.17.0"
    assert result["image"] == image and result["compose_project"] == "fr3_stack_jz"
    assert result["labels"]["org.fr3-stack.variant"] == "jz"
    assert result["release_dir"] == str(tmp_path / "jz" / ("a" * 64))
    verify.assert_called_once_with(tmp_path / "jz" / ("a" * 64), "a" * 64)
    build_argv = invocations[0]
    assert build_argv[build_argv.index("-t") + 1] == image
    assert "BUILD_JOBS=2" in build_argv
    probe = help_probe.call_args.args[0]
    assert probe[-2:] == [image, "--help"]
    assert probe[probe.index("--network") + 1] == "none"
    assert not result["daemon_restarted"]


@pytest.mark.parametrize("args", [["--jz", "ps"], ["config", "--jz"], ["ft", "logs", "--jz"]])
def test_jz_launcher_targets_its_own_compose_project(tmp_path, args):
    executable = tmp_path / "docker"
    executable.write_text("#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n")
    executable.chmod(0o755)
    env = {**os.environ, "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"]}
    result = subprocess.run(["bash", str(ROOT / "fr3-stack"), *args], env=env, capture_output=True, text=True, check=True)
    argv = json.loads(result.stdout)
    assert argv[argv.index("--project-name") + 1] == "fr3_stack_jz"
    assert "containers/compose.jz.yml" in argv and "--jz" not in argv
    if args[0] == "ft":
        assert argv[-2:] == ["logs", "fr3-stack-ft"]


def test_failed_build_does_not_install_client(tiny_repo, tmp_path, monkeypatch):
    config = tmp_path / "config.json"
    value = json.loads((ROOT / "configs/nuc-sync.example.json").read_text())
    value["python"] = "/usr/bin/python3"
    config.write_text(json.dumps(value))
    monkeypatch.setattr(sync, "ROOT", tiny_repo)
    monkeypatch.setattr(sync, "command", lambda *_args, **_kw: "base")
    monkeypatch.setattr(sync, "client_info", lambda _: {"module": "old"})
    monkeypatch.setattr(remote, "inspect", lambda _: {"build_blockers": []})
    monkeypatch.setattr(sync, "snapshot", lambda *_: (tmp_path, tmp_path / "source.tar.gz", {"tree_sha256": "a" * 64, "files": []}))
    monkeypatch.setattr(remote, "build", Mock(side_effect=RuntimeError("compiler failed")))
    installer = Mock()
    monkeypatch.setattr(sync, "install_client", installer)
    assert sync.main(["build-local", "--config", str(config)]) == 2
    installer.assert_not_called()
    record = json.loads((tmp_path / "nuc-sync/latest.json").read_text())
    assert not record["nuc_contacted"]
    assert not record["client_updated"]


def test_failed_client_validation_reinstalls_old_source(tmp_path, monkeypatch):
    old = tmp_path / "old"
    old.mkdir()
    (old / "pyproject.toml").write_text("[project]\n")
    monkeypatch.setattr(sync, "client_info", lambda _: {"module": str(old / "fr3_stack/__init__.py")})
    commands = []
    def invoke(argv, **_kwargs):
        commands.append(argv)
        if len(commands) == 3:
            raise RuntimeError("new dependency mismatch")
        return ""
    monkeypatch.setattr(sync, "command", invoke)
    with pytest.raises(RuntimeError, match="dependency mismatch"):
        sync.install_client("python", tmp_path / "new", {})
    assert commands[-1][-1] == str(old)
