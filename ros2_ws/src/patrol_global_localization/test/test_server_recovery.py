"""Map-package crash recovery tests for the web server."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


PROJECT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "patrol_web_server_for_test", PROJECT / "patrol_web" / "server.py")
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


def write_package(directory, map_id, marker, complete=True):
    directory.mkdir()
    (directory / "map.pcd").write_bytes(marker.encode() + b"x" * 500)
    if complete:
        (directory / "map.npy").write_bytes(b"npy")
        (directory / "metadata.json").write_text(json.dumps({
            "map_id": map_id,
            "pcd": "map.pcd",
            "numpy_map": "map.npy",
            "database": None,
            "trajectory": None,
            "nav_map": None,
        }), encoding="utf-8")


def test_recover_complete_first_save_staging(tmp_path, monkeypatch):
    monkeypatch.setattr(SERVER, "DATA", tmp_path)
    maps = tmp_path / "maps"
    maps.mkdir()
    staging = maps / ".staging_factory_a_123"
    write_package(staging, "factory_a", "new")

    SERVER.recover_map_dirs()

    assert not staging.exists()
    assert (maps / "factory_a" / "metadata.json").is_file()


def test_discard_incomplete_first_save_staging(tmp_path, monkeypatch):
    monkeypatch.setattr(SERVER, "DATA", tmp_path)
    maps = tmp_path / "maps"
    maps.mkdir()
    staging = maps / ".staging_factory_a_123"
    write_package(staging, "factory_a", "partial", complete=False)

    SERVER.recover_map_dirs()

    assert not staging.exists()
    assert not (maps / "factory_a").exists()


def test_overwrite_crash_restores_backup_not_staging(tmp_path, monkeypatch):
    monkeypatch.setattr(SERVER, "DATA", tmp_path)
    maps = tmp_path / "maps"
    maps.mkdir()
    backup = maps / ".old_factory_a_456"
    staging = maps / ".staging_factory_a_123"
    write_package(backup, "factory_a", "old")
    write_package(staging, "factory_a", "new")

    SERVER.recover_map_dirs()

    assert (maps / "factory_a" / "map.pcd").read_bytes().startswith(b"old")
    assert not backup.exists()
    assert not staging.exists()


def test_active_map_never_uses_another_maps_grid(tmp_path, monkeypatch):
    monkeypatch.setattr(SERVER, "DATA", tmp_path)
    monkeypatch.setattr(SERVER, "ACTIVE_MAP_FILE", tmp_path / "active_map.json")
    maps = tmp_path / "maps"
    (maps / "map_a").mkdir(parents=True)
    (maps / "map_b").mkdir()
    (maps / "map_b" / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
    SERVER.write_json(SERVER.ACTIVE_MAP_FILE, {"name": "map_a"})

    assert SERVER.latest_nav_map() is None


def test_select_packaged_map_requires_its_own_grid(tmp_path, monkeypatch):
    monkeypatch.setattr(SERVER, "DATA", tmp_path)
    monkeypatch.setattr(SERVER, "ACTIVE_MAP_FILE", tmp_path / "active_map.json")
    package = tmp_path / "maps" / "map_a"
    package.mkdir(parents=True)
    (package / "map.pcd").write_bytes(b"x" * 500)
    (package / "metadata.json").write_text(
        json.dumps({"localization_ready": True, "coord_version": "v1"}),
        encoding="utf-8")

    with pytest.raises(ValueError, match="导航层"):
        SERVER.select_map("map_a")


def test_projection_failure_does_not_activate_map(tmp_path, monkeypatch):
    data = tmp_path / "data"
    runtime = tmp_path / "runtime"
    cloud = runtime / "cloud"
    (data / "maps").mkdir(parents=True)
    cloud.mkdir(parents=True)
    paths = {
        "DATA": data,
        "RUNTIME": runtime,
        "CLOUD_RUNTIME": cloud,
        "PROJECT": tmp_path,
        "CLOUD_SAVE_REQUEST": cloud / "request.json",
        "CLOUD_SAVE_RESPONSE": cloud / "response.json",
        "MAPPING_SESSION_FILE": runtime / "mapping_session.json",
        "CHECKPOINTS_FILE": data / "checkpoints.json",
        "ACTIVE_MAP_FILE": data / "active_map.json",
    }
    for name, value in paths.items():
        monkeypatch.setattr(SERVER, name, value)
    session_id = "12345678abcdef"
    SERVER.write_json(SERVER.MAPPING_SESSION_FILE, {"id": session_id})
    SERVER.write_json(SERVER.CHECKPOINTS_FILE, [])
    (runtime / "trajectory_12345678.json").write_text(
        '{"keyframes":[{},{},{}]}', encoding="utf-8")
    original_write = SERVER.write_json

    def write_hook(path, value):
        original_write(path, value)
        if Path(path) == SERVER.CLOUD_SAVE_REQUEST:
            (data / "maps" / "map_a.pcd").write_bytes(b"x" * 500)
            np.save(data / "maps" / "map_a.npy",
                    np.zeros((200, 3), dtype=np.float32))
            original_write(SERVER.CLOUD_SAVE_RESPONSE, {
                "id": value["id"], "ok": True, "name": "map_a", "points": 200})

    def fake_run(command, **_kwargs):
        staging = data / "maps" / f".staging_map_a_{SERVER.os.getpid()}"
        if "pcd_to_nav_map.py" in " ".join(command):
            return SimpleNamespace(returncode=1, stdout="", stderr="projection failed")
        np.savez_compressed(staging / "db.npz", poses=np.zeros((3, 8)),
                            descriptors=np.zeros((3, 1200)),
                            sc_shape=np.array([20, 60]))
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(SERVER, "write_json", write_hook)
    monkeypatch.setattr(SERVER.subprocess, "run", fake_run)
    result = SERVER.save_cloud_map("map_a")

    assert result["localization_ready"] is True
    assert result.get("nav_error")
    assert not SERVER.ACTIVE_MAP_FILE.exists()
