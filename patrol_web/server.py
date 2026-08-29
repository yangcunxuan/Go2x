#!/usr/bin/env python3
"""Go2 patrol console: device status, maps, checkpoints and block tasks."""

import json
import math
import mimetypes
import os
import signal
import struct

import numpy as np
import subprocess
import threading
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

ROOT = Path(__file__).resolve().parent
PROJECT = Path(os.environ.get("PATROL_PROJECT", ROOT.parent))
DATA = Path(os.environ.get("PATROL_DATA", PROJECT / "patrol_data"))
RUNTIME = Path(os.environ.get("PATROL_RUNTIME", PROJECT / "runtime"))
CLOUD_RUNTIME = Path(os.environ.get("PATROL_CLOUD_RUNTIME", RUNTIME / "cloud_bridge"))
STATIC = ROOT / "static"
HOST = os.environ.get("PATROL_HTTP_HOST", "127.0.0.1")
PORT = int(os.environ.get("PATROL_HTTP_PORT", "8090"))
C12_API = os.environ.get("C12_API", "http://127.0.0.1:8088")

for directory in (DATA, RUNTIME, DATA / "maps", DATA / "tasks"):
    directory.mkdir(parents=True, exist_ok=True)

CONFIG_FILE = DATA / "config.json"
CHECKPOINTS_FILE = DATA / "checkpoints.json"
TASKS_FILE = DATA / "tasks.json"
STATE_FILE = RUNTIME / "robot_state.json"
MAP_FILE = RUNTIME / "map.json"
CLOUD_FILE = CLOUD_RUNTIME / "cloud.json"
CLOUD_SAVE_REQUEST = CLOUD_RUNTIME / "cloud_save_request.json"
CLOUD_SAVE_RESPONSE = CLOUD_RUNTIME / "cloud_save_response.json"
GOAL_FILE = RUNTIME / "goal.json"
TELEOP_FILE = RUNTIME / "teleop.json"
NAV_ENABLE_FILE = RUNTIME / "nav_motion_enable.json"
NAV_PATH_FILE = RUNTIME / "nav_path.json"
GO2_STATE_FILE = RUNTIME / "go2_state.json"
GO2_ACTION_FILE = RUNTIME / "go2_action.json"
GO2_NAV_COMMAND_FILE = RUNTIME / "go2_nav_command.json"
ACTIVE_MAP_FILE = DATA / "active_map.json"
MAPPING_SESSION_FILE = RUNTIME / "mapping_session.json"
MAPPING_RECOVERY_FILE = CLOUD_RUNTIME / "mapping_recovery.json"

DEFAULT_CONFIG = {
    "sensor": "mid360",
    "robot_name": "GO2X",
    "scan_topic": "/scan",
    "odom_topic": "/Odometry",
    "cmd_vel_topic": "/cmd_vel",
    "map_frame": "map_level",
    "base_frame": "body",
}

LOCK = threading.RLock()
SERVICES = {}
SERVICE_SCRIPTS = {
    "mapping": "run_mid360_nav_mapping_stack.sh",
    "navigation": "run_navigation_stack_v2.sh",
    "go2_state": "run_go2_state_bridge.sh",
}
TASK_RUN = {"id": None, "task_id": None, "state": "idle", "step": 0, "message": "", "started_at": None}
TASK_CANCEL = threading.Event()
TELEOP = {"armed": True, "active_key": None, "updated_at": 0.0, "message": "直接控制 · 按住运动，松开停止"}
TELEOP_DIRECTIONS = {
    "u": (1.0, 1.0, 0.0), "i": (1.0, 0.0, 0.0), "o": (1.0, -1.0, 0.0),
    "j": (0.0, 1.0, 0.0), "k": (0.0, 0.0, 0.0), "l": (0.0, -1.0, 0.0),
    "m": (-1.0, 1.0, 0.0), ",": (-1.0, 0.0, 0.0), ".": (-1.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0), "e": (0.0, 0.0, -1.0),
}


def read_json(path, default):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return default


def write_json(path, value):
    # Unique tmp name per process: the planner-motion bridge writes the same
    # command files from inside the container, and two writers sharing one
    # ".tmp" race each other into FileNotFoundError on os.replace.
    temporary = Path(f"{path}.{os.getpid()}.tmp")
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, path)


def teleop_command(key="k", speed=0.0, force_stop=False):
    """Write one short-lived velocity command consumed by patrol_bridge."""
    key = str(key).lower()
    if key not in TELEOP_DIRECTIONS:
        raise ValueError("不支持的控制按键")
    speed = max(0.05, min(0.60, float(speed))) if not force_stop else 0.0
    x, y, yaw = TELEOP_DIRECTIONS[key]
    if key in ("u", "o", "m", "."):
        x *= 0.7071; y *= 0.7071
    command = {
        "id": uuid.uuid4().hex, "updated_at": time.time(),
        "key": "k" if force_stop else key,
        "vx": 0.0 if force_stop else x * speed,
        "vy": 0.0 if force_stop else y * speed,
        "vyaw": 0.0 if force_stop else yaw * min(1.00, speed * 2.0),
    }
    write_json(TELEOP_FILE, command)
    # Manual web control must cross from the web process to GO2 DDS domain 0,
    # just like autonomous navigation.  The old patrol_bridge publisher lives
    # with FAST-LIO in domain 42 and cannot directly reach the robot after the
    # MID360 network separation.
    if force_stop or key == "k":
        write_json(GO2_NAV_COMMAND_FILE, {"api_id": 1003, "parameter": "", "source": "teleop",
                                          "updated_at": time.time()})
    else:
        parameter = json.dumps({"x": command["vx"], "y": command["vy"], "z": command["vyaw"]},
                               ensure_ascii=False, separators=(",", ":"))
        write_json(GO2_NAV_COMMAND_FILE, {"api_id": 1008, "parameter": parameter, "source": "teleop",
                                          "updated_at": command["updated_at"]})
    TELEOP.update(active_key=None if force_stop or key == "k" else key,
                  updated_at=command["updated_at"],
                  message="已停止" if force_stop or key == "k" else f"按住 {key.upper()} 运动")
    return command


def teleop_disarm(message="控制已锁定"):
    teleop_command(force_stop=True)
    TELEOP.update(armed=True, active_key=None, message=message)


def config():
    result = dict(DEFAULT_CONFIG)
    result.update(read_json(CONFIG_FILE, {}))
    return result


def service_record(name):
    return RUNTIME / f"service_{name}.json"


def pid_alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def service_status():
    result = {}
    with LOCK:
        for name in SERVICE_SCRIPTS:
            item = SERVICES.get(name)
            record = read_json(service_record(name), {})
            pid = item["process"].pid if item else record.get("pid")
            if item:
                code = item["process"].poll(); running = code is None
            else:
                code = record.get("returncode"); running = pid_alive(pid)
            if pid:
                result[name] = {"running": running, "pid": int(pid),
                                "started_at": (item or {}).get("started_at", record.get("started_at")),
                                "returncode": None if running else code,
                                "persistent": True}
    return result


def start_service(name, script):
    with LOCK:
        existing = service_status().get(name, {})
        if existing.get("running"): return existing
        log_dir = RUNTIME / "logs"
        log_dir.mkdir(exist_ok=True)
        log = open(log_dir / f"{name}.log", "wb", buffering=0)
        process = subprocess.Popen(
            ["bash", str(PROJECT / "scripts" / script)],
            cwd=PROJECT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        SERVICES[name] = {"process": process, "started_at": time.strftime("%F %T"), "log": log}
        write_json(service_record(name), {"pid": process.pid, "pgid": process.pid, "script": script,
                                          "started_at": SERVICES[name]["started_at"], "persistent": True})
        return {"running": True, "pid": process.pid, "started_at": SERVICES[name]["started_at"], "persistent": True}


def require_service_stays_running(name, seconds=4.0):
    deadline = time.time() + seconds
    while time.time() < deadline:
        with LOCK:
            item = SERVICES.get(name)
            code = item["process"].poll() if item else -1
        if code is not None:
            log = RUNTIME / "logs" / f"{name}.log"
            detail = log.read_text(encoding="utf-8", errors="replace")[-800:] if log.exists() else ""
            raise RuntimeError(f"{name}启动失败（退出码{code}）：{detail.strip()}")
        time.sleep(0.2)
    return service_status()[name]


def stop_service(name):
    with LOCK:
        item = SERVICES.get(name)
        record = read_json(service_record(name), {})
        pid = item["process"].pid if item else record.get("pid")
        if not pid or not pid_alive(pid): return False
        try:
            os.killpg(int(record.get("pgid", pid)), signal.SIGINT)
            if item:
                try: item["process"].wait(timeout=6)
                except subprocess.TimeoutExpired: os.killpg(int(record.get("pgid", pid)), signal.SIGTERM)
            else:
                deadline = time.time() + 6
                while time.time() < deadline and pid_alive(pid): time.sleep(0.1)
                if pid_alive(pid): os.killpg(int(record.get("pgid", pid)), signal.SIGTERM)
        except (ProcessLookupError, PermissionError): pass
        write_json(service_record(name), {**record, "pid": int(pid), "stopped_at": time.strftime("%F %T"),
                                          "returncode": 0, "persistent": True})
        return True


def save_map(name):
    safe = "".join(char for char in name if char.isalnum() or char in "_-" or "\u4e00" <= char <= "\u9fff")[:40]
    if not safe:
        safe = time.strftime("map_%Y%m%d_%H%M%S")
    target = f"/project/patrol_data/maps/{safe}"
    command = ["docker-compose", "run", "--rm"]
    if config()["sensor"] == "go2":
        command += ["-e", "ROS_DOMAIN_ID=0", "-e", "CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml"]
    command += ["ros2", "bash", "-lc",
                f"source /opt/ros/humble/setup.bash; ros2 run nav2_map_server map_saver_cli -f '{target}'"]
    result = subprocess.run(
        command,
        cwd=PROJECT, capture_output=True, text=True, timeout=30,
    )
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout or "地图保存失败")[-500:])
    return {"name": safe, "yaml": target + ".yaml", "pgm": target + ".pgm"}


def save_cloud_map(name):
    safe = "".join(char for char in name if char.isalnum() or char in "_-" or "\u4e00" <= char <= "\u9fff")[:40]
    if not safe:
        safe = time.strftime("map_%Y%m%d_%H%M%S")
    # P0 audit #2: session must be read before anything that uses it.
    session = read_json(MAPPING_SESSION_FILE, {})
    session_id = session.get("id", "")
    request_id = uuid.uuid4().hex
    write_json(CLOUD_SAVE_REQUEST, {"id": request_id, "name": safe, "created_at": time.time()})
    deadline = time.time() + 15
    while time.time() < deadline:
        response = read_json(CLOUD_SAVE_RESPONSE, {})
        if response.get("id") != request_id:
            time.sleep(0.15)
            continue
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "三维地图保存失败"))
        # The bridge wrote maps/<safe>.pcd + maps/<safe>.npy (map_level).
        pcd_flat = DATA / "maps" / f"{safe}.pcd"
        npy_flat = DATA / "maps" / f"{safe}.npy"
        if not pcd_flat.is_file() or pcd_flat.stat().st_size < 400:
            raise RuntimeError("保存的点云为空或过小，已中止")
        map_id = safe
        staging = DATA / "maps" / f".staging_{map_id}_{os.getpid()}"
        if staging.exists():
            shutil.rmtree(staging)
        staging.mkdir(parents=True)
        shutil.move(str(pcd_flat), str(staging / "map.pcd"))
        if npy_flat.is_file():
            shutil.move(str(npy_flat), str(staging / "map.npy"))
        # 2D navigation layer (PGM/YAML) from the PCD; failure keeps the map
        # viewable but not nav-grid-checked.
        projection = subprocess.run(
            ["docker-compose", "run", "--rm", "ros2", "python3",
             "/project/scripts/pcd_to_nav_map.py", str(staging / "map.pcd")],
            cwd=PROJECT, capture_output=True, text=True, timeout=120)
        nav_ok = projection.returncode == 0
        if not nav_ok:
            response["nav_error"] = (projection.stderr or projection.stdout or "导航层生成失败")[-500:]
        # Keyframe trajectory recorded by keyframe_saver during the session.
        trajectory_src = RUNTIME / f"trajectory_{session_id[:8]}.json"
        trajectory_ok = trajectory_src.is_file()
        if trajectory_ok:
            shutil.copy(str(trajectory_src), str(staging / "trajectory.json"))
        # Plan A database: descriptors + keyframe poses from map.npy.
        localization_ready = False
        if trajectory_ok:
            db_run = subprocess.run(
                ["docker-compose", "run", "--rm", "ros2", "bash", "-c",
                 "PYTHONPATH=/project/ros2_ws/src/patrol_global_localization "
                 "python3 -m patrol_global_localization.build_map_db "
                 f"/project/patrol_data/maps/.staging_{map_id}_{os.getpid()}/map.npy "
                 f"/project/patrol_data/maps/.staging_{map_id}_{os.getpid()}/trajectory.json"],
                cwd=PROJECT, capture_output=True, text=True, timeout=600)
            db_file = staging / "db.npz"
            if db_run.returncode == 0 and db_file.is_file():
                try:
                    db = np.load(db_file)
                    localization_ready = len(db["poses"]) >= 3 and db["descriptors"].shape[0] >= 3
                except (OSError, ValueError, KeyError):
                    localization_ready = False
            if not localization_ready:
                response["db_error"] = (db_run.stderr or db_run.stdout or "关键帧数据库生成失败")[-300:]
        metadata = {
            "map_id": map_id,
            "frame": "map_level",
            "pcd": "map.pcd",
            "numpy_map": "map.npy" if (staging / "map.npy").is_file() else None,
            "database": "db.npz" if (staging / "db.npz").is_file() else None,
            "trajectory": "trajectory.json" if trajectory_ok else None,
            "nav_map": "map.yaml" if nav_ok else None,
            "lidar_extrinsic_version": "mid360_mount_v1",
            "session_id": session_id,
            "points": int(response.get("points", 0)),
            "created_at": time.time(),
            "mapping_available": True,
            "localization_ready": localization_ready,
        }
        write_json(staging / "metadata.json", metadata)
        # Atomic finalize: the package directory appears complete or not at all.
        final = DATA / "maps" / map_id
        if final.exists():
            shutil.rmtree(final)
        os.replace(staging, final)
        # Bind checkpoints created during this session to the map.
        points_list = read_json(CHECKPOINTS_FILE, [])
        changed = False
        for point in points_list:
            if session_id and point.get("session_id") == session_id:
                point["map_name"] = map_id
                changed = True
        if changed:
            write_json(CHECKPOINTS_FILE, points_list)
        # Only a localization-ready map becomes the ACTIVE navigation map;
        # an incomplete package stays viewable but cannot be navigated to.
        if localization_ready:
            write_json(ACTIVE_MAP_FILE, {"name": map_id, "updated_at": time.time()})
        response["map_id"] = map_id
        response["localization_ready"] = localization_ready
        response["session_id"] = session_id
        return response
    raise RuntimeError("等待三维地图保存超时")


def c12_status():
    try:
        with urllib.request.urlopen(C12_API + "/api/status", timeout=1.0) as response:
            return json.load(response)
    except Exception:
        return {"visible": False, "thermal": False}


def c12_presets():
    try:
        with urllib.request.urlopen(C12_API + "/api/presets", timeout=1.5) as response:
            return json.load(response).get("presets", [])
    except Exception:
        return []


def capture_c12(task_name):
    snapshot_dir = DATA / "snapshots" / time.strftime("%Y%m%d")
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%H%M%S") + "_" + uuid.uuid4().hex[:6]
    files = {}
    for camera in ("visible", "thermal"):
        target = snapshot_dir / f"{stamp}_{camera}.jpg"
        with urllib.request.urlopen(C12_API + f"/snapshot/{camera}", timeout=3) as response:
            target.write_bytes(response.read())
        files[camera] = str(target)
    write_json(snapshot_dir / f"{stamp}.json", {"task": task_name, "time": time.time(), "files": files})
    return files


def robot_state():
    state = read_json(STATE_FILE, {})
    updated = float(state.get("updated_at", 0))
    state["bridge_online"] = bool(updated and time.time() - updated < 3)
    if not state["bridge_online"]:
        state["odom_online"] = False
    # The large accumulated Laser_map is handled by a dedicated DDS participant
    # pinned to the wired robot network.  Its freshness must not inherit the
    # control bridge's network-interface state.
    cloud = read_json(CLOUD_FILE, {})
    cloud_updated = float(cloud.get("updated_at", 0))
    state["cloud_map_age"] = time.time() - cloud_updated if cloud_updated else None
    state["cloud_map_online"] = bool(cloud.get("available") and cloud_updated and time.time() - cloud_updated < 3)
    state.setdefault("pose", {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0})
    pose = state["pose"]
    values = [float(pose.get(axis, 0)) for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
    bridge_sane = bool(state.get("localization_sane", True))
    sane = bridge_sane and all(math.isfinite(value) for value in values) and abs(values[0]) < 200 and abs(values[1]) < 200 and abs(values[2]) < 20
    # Cross-check FAST-LIO height against the independent GO2 body state when
    # this session is aligned to a saved map.  A down-to-stand transition moves
    # MID360 only about the body-height delta; metre-scale Z drift is localization
    # failure even when it accumulated too slowly for the per-frame jump guard.
    alignment = read_json(RUNTIME / "localization_alignment.json", {})
    go2_for_height = read_json(GO2_STATE_FILE, {})
    # The alignment-file height cross-check is only meaningful in MAPPING
    # mode (fixed leveling TF). In localization mode the dynamic TF owns the
    # map_level height and a stale alignment z would false-positive.
    if (sane and "z" in alignment
            and service_status().get("mapping", {}).get("running")
            and time.time() - float(go2_for_height.get("updated_at", 0)) < 2):
        reference_height = float(alignment.get("reference_body_height", 0.07))
        expected_z = float(alignment["z"]) + float(go2_for_height.get("body_height", reference_height)) - reference_height
        if abs(values[2] - expected_z) > 0.45:
            sane = False
            state["localization_error"] = f"FAST-LIO高度与GO2机身高度不一致：定位z={values[2]:.2f}m，预期约{expected_z:.2f}m"
    state["localization_sane"] = sane
    state["localization_error"] = None if sane else (state.get("localization_error") or "FAST-LIO位姿超出安全范围，疑似定位发散")
    state["online"] = bool(state["bridge_online"] and state.get("odom_online", False) and sane)
    state.setdefault("topics", [])
    # Action status is written by a ROS subscriber and may remain "navigating"
    # after the navigation container is stopped.  Service state is authoritative.
    if not service_status().get("navigation", {}).get("running"):
        state["nav_status"] = "stopped"
    return state


def mapping_session():
    session = read_json(MAPPING_SESSION_FILE, {})
    session["running"] = bool(service_status().get("mapping", {}).get("running"))
    return session


def go2_state(max_age=3.0):
    state = read_json(GO2_STATE_FILE, {})
    updated = float(state.get("updated_at", 0))
    # max_age: teleop passes a wider window. Under mapping load the state
    # bridge's 0.5 s write can lag past 3 s even though the dog is fine; the
    # authoritative temperature/height gates run inside the state bridge on
    # live values, so the web-side freshness check is only a duplicate gate.
    state["online"] = bool(updated and time.time() - updated < max_age and
                           (state.get("lowstate_online") or state.get("sportstate_online")))
    state["age"] = round(time.time() - updated, 2) if updated else None
    return state


def require_go2_motion_safe(require_standing=True, max_age=3.0):
    state = go2_state(max_age=max_age)
    if not state.get("online"):
        raise ValueError("GO2状态离线，禁止运动")
    temperature = state.get("max_motor_temperature_c")
    if temperature is None or float(temperature) >= 85:
        raise ValueError(f"电机温度过高（{temperature}°C），测试保护阈值为85°C")
    if require_standing and float(state.get("body_height", 0)) < 0.18:
        raise ValueError("机器狗当前未站立，禁止导航或移动")
    return state


def issue_goal(checkpoint):
    goal = {
        "id": uuid.uuid4().hex,
        "type": "navigate",
        "x": checkpoint["x"], "y": checkpoint["y"], "z": checkpoint.get("z", 0),
        "yaw": checkpoint.get("yaw", 0),
        "checkpoint_id": checkpoint["id"],
        "created_at": time.time(),
        # Goal lifecycle: the relay stops publishing past this timestamp, so
        # a stale goal can never keep planning after a failure or restart.
        "expires_at": time.time() + 600,
    }
    write_json(GOAL_FILE, goal)
    return goal


def speak_navigation(text):
    """Speak navigation state through the logged-in Ubuntu user's audio.

    espeak-ng with the cmn (Mandarin) voice directly — speech-dispatcher has
    no Chinese voice configured here and falls back to English phonemes,
    which sounded like broken English.
    """
    try:
        environment = os.environ.copy()
        environment.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
        subprocess.Popen(
            ["espeak-ng", "-v", "cmn", "-s", "140", str(text)], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except OSError:
        pass


def monitor_navigation_announcement(goal_id, baseline_publish_count):
    """Announce only after actual motion starts; fail closed on planner errors."""
    announced = False
    startup_deadline = time.monotonic() + 20.0
    while True:
        current = read_json(GOAL_FILE, {})
        if current.get("id") != goal_id:
            return
        state = robot_state()
        nav_status = state.get("nav_status")
        running = service_status().get("navigation", {}).get("running")
        relay = go2_state().get("nav_relay", {})
        publish_count = int(relay.get("publish_count", 0) or 0)
        if not running or nav_status in ("failed", "canceled"):
            set_nav_motion(False, "导航失败，运动已切断")
            speak_navigation("导航失败")
            safety = go2_state()
            if float(safety.get("body_height", 0) or 0) >= 0.18:
                write_json(GO2_ACTION_FILE, {
                    "id": uuid.uuid4().hex, "api_id": 1005,
                    "action": "stand_down", "created_at": time.time(),
                })
            return
        if not announced and publish_count > baseline_publish_count:
            speak_navigation("导航已启动")
            announced = True
        if nav_status == "succeeded":
            return
        if not announced and time.monotonic() >= startup_deadline:
            set_nav_motion(False, "导航启动超时，运动已切断")
            speak_navigation("导航失败")
            safety = go2_state()
            if float(safety.get("body_height", 0) or 0) >= 0.18:
                write_json(GO2_ACTION_FILE, {
                    "id": uuid.uuid4().hex, "api_id": 1005,
                    "action": "stand_down", "created_at": time.time(),
                })
            return
        time.sleep(0.25)


def set_nav_motion(enabled, reason=""):
    state = {
        "enabled": bool(enabled), "reason": str(reason), "updated_at": time.time(),
        "expires_at": time.time() + 600 if enabled else 0,
    }
    write_json(NAV_ENABLE_FILE, state)
    return state


def active_map_id():
    """The single authority for "which map is active": active_map.json.
    Never derive the id from a YAML/PCD file name (P0 audit #2/#4)."""
    return read_json(ACTIVE_MAP_FILE, {}).get("name")


def latest_nav_map():
    """2D nav grid (YAML) of the active map. Package layout first
    (maps/<map_id>/map.yaml), then the legacy flat layout."""
    active = read_json(ACTIVE_MAP_FILE, {}).get("name")
    if active:
        packaged = DATA / "maps" / active / "map.yaml"
        if packaged.is_file():
            return packaged
        legacy = DATA / "maps" / (Path(active).stem + ".yaml")
        if legacy.is_file():
            return legacy
    maps = sorted((DATA / "maps").glob("*/map.yaml"),
                  key=lambda path: path.stat().st_mtime, reverse=True)
    if maps:
        return maps[0]
    maps = sorted((DATA / "maps").glob("*.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)
    return maps[0] if maps else None


def map_inventory():
    active = read_json(ACTIVE_MAP_FILE, {}).get("name")
    result = []
    for meta_path in sorted((DATA / "maps").glob("*/metadata.json"),
                            key=lambda p: p.stat().st_mtime, reverse=True):
        map_id = meta_path.parent.name
        meta = read_json(meta_path, {})
        pcd = meta_path.parent / "map.pcd"
        session_id = read_json(MAPPING_SESSION_FILE, {}).get("id")
        result.append({
            "name": map_id, "pcd": "map.pcd",
            "points": meta.get("points", 0),
            "bytes": pcd.stat().st_size if pcd.is_file() else 0,
            "modified_at": meta_path.stat().st_mtime,
            "nav_ready": (meta_path.parent / "map.yaml").is_file(),
            "localization_ready": bool(meta.get("localization_ready")),
            "active": map_id == active,
            "session_id": meta.get("session_id"),
            "current_session": bool(session_id and meta.get("session_id") == session_id),
            "legacy": False,
        })
    for pcd in sorted((DATA / "maps").glob("*.pcd"), key=lambda p: p.stat().st_mtime, reverse=True):
        if pcd.stem in {item["name"] for item in result}:
            continue
        points = None
        try:
            with open(pcd, "rb") as handle:
                for _ in range(40):
                    line = handle.readline().decode("ascii", "ignore").strip()
                    if line.startswith("POINTS "):
                        points = int(line.split()[1])
                    if line.startswith("DATA "):
                        break
        except (OSError, ValueError):
            pass
        meta = read_json(pcd.with_suffix(".meta.json"), {})
        session_id = read_json(MAPPING_SESSION_FILE, {}).get("id")
        result.append({"name": pcd.stem, "pcd": pcd.name, "points": points,
                       "bytes": pcd.stat().st_size, "modified_at": pcd.stat().st_mtime,
                       "nav_ready": pcd.with_suffix(".yaml").is_file(),
                       "localization_ready": False,
                       "active": pcd.stem == active,
                       "session_id": meta.get("session_id"),
                       "current_session": bool(session_id and meta.get("session_id") == session_id),
                       "legacy": True})
    return result


def select_map(name):
    safe = Path(str(name)).stem
    packaged = DATA / "maps" / safe / "map.pcd"
    legacy = DATA / "maps" / (safe + ".pcd")
    if packaged.is_file():
        meta = read_json(DATA / "maps" / safe / "metadata.json", {})
        if not meta.get("localization_ready", False):
            raise ValueError("该地图缺少重定位数据库，不能激活为导航地图")
    elif legacy.is_file():
        if not legacy.with_suffix(".yaml").is_file():
            raise ValueError("该PCD尚未生成导航层")
    else:
        raise ValueError("三维地图不存在")
    write_json(ACTIVE_MAP_FILE, {"name": safe, "updated_at": time.time()})
    return {"ok": True, "name": safe}


def load_pcd_cloud(name):
    safe = Path(str(name)).stem
    # Packaged maps: load the NumPy twin directly (exact float32, no parser).
    npy = DATA / "maps" / safe / "map.npy"
    if npy.is_file():
        meta = read_json(DATA / "maps" / safe / "metadata.json", {})
        points = np.load(npy, mmap_mode="r")
        available = int(points.shape[0])
        stride = max(1, math.ceil(available / 24000))
        selected_array = np.asarray(points[::stride], dtype=np.float64)
        low = [float(v) for v in selected_array.min(axis=0)]
        high = [float(v) for v in selected_array.max(axis=0)]
        selected = [round(float(v), 3) for v in selected_array.reshape(-1)]
        if len(selected) > 72000:
            selected = selected[:72000]
        return {"available": bool(len(selected)), "frame": "map_level",
                "map_name": safe, "total_points": available,
                "shown_points": len(selected) // 3, "bounds": [low, high],
                "points": selected, "updated_at": npy.stat().st_mtime}
    target = DATA / "maps" / (safe + ".pcd")
    if not target.is_file() or target.parent != (DATA / "maps"):
        raise ValueError("三维地图不存在")
    with open(target, "rb") as handle:
        fields = []; sizes = []; types = []; count = 0; mode = ""
        for _ in range(60):
            line = handle.readline().decode("ascii", "ignore").strip()
            if line.startswith("FIELDS "): fields = line.split()[1:]
            elif line.startswith("SIZE "): sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE "): types = line.split()[1:]
            elif line.startswith("POINTS "): count = int(line.split()[1])
            elif line.startswith("DATA "): mode = line.split()[1]; break
        if mode != "binary" or fields[:3] != ["x", "y", "z"] or sizes[:3] != [4, 4, 4] or types[:3] != ["F", "F", "F"]:
            raise ValueError("目前仅支持系统保存的XYZ二进制PCD")
        raw = handle.read()
    available = min(count, len(raw) // 12)
    stride = max(1, math.ceil(available / 24000))
    selected = []
    low = [float("inf")] * 3; high = [float("-inf")] * 3
    for index in range(0, available, stride):
        xyz = struct.unpack_from("<fff", raw, index * 12)
        if not all(math.isfinite(v) for v in xyz): continue
        selected.extend(round(v, 3) for v in xyz)
        for axis, value in enumerate(xyz): low[axis] = min(low[axis], value); high[axis] = max(high[axis], value)
        if len(selected) >= 72000: break
    return {"available": bool(selected), "frame": "map_level", "map_name": safe,
            "total_points": available, "shown_points": len(selected) // 3,
            "bounds": [low, high], "points": selected, "updated_at": target.stat().st_mtime}


def stop_radar_data():
    set_nav_motion(False, "MID360数据已停止")
    teleop_disarm("雷达停止，控制已锁定")
    autosave = None; save_error = None
    state = robot_state()
    if service_status().get("mapping", {}).get("running") and state.get("online") and state.get("cloud_map_online"):
        try: autosave = save_cloud_map("autosave_radar_stop_" + time.strftime("%Y%m%d_%H%M%S"))
        except Exception as exc: save_error = str(exc)
    stopped = []
    for name in ("navigation", "mapping"):
        if stop_service(name): stopped.append(name)
    listing = subprocess.run(["docker", "ps", "-q", "--filter", "ancestor=go2-mid360:humble"],
                             capture_output=True, text=True, timeout=5)
    for container in listing.stdout.split():
        top = subprocess.run(["docker", "top", container, "-eo", "args"], capture_output=True, text=True, timeout=5).stdout
        if any(token in top for token in ("inside_mid360_mapping.sh", "inside_navigation.sh", "fastlio_mapping")):
            subprocess.run(["docker", "stop", "-t", "5", container], capture_output=True, timeout=10)
            stopped.append(container[:12])
    return {"stopped": stopped, "power_off": False, "autosave": autosave, "save_error": save_error,
            "message": "MID360采集、FAST-LIO和导航进程已停止；硬件仍通电，如需断电需加受控电源开关。"}


def load_nav_grid(yaml_path):
    values = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    resolution = float(values["resolution"])
    origin = json.loads(values["origin"])
    image_path = yaml_path.parent / values["image"]
    raw = image_path.read_bytes()
    magic, dimensions, maximum, raster = raw.split(b"\n", 3)
    if magic != b"P5" or maximum.strip() != b"255":
        raise ValueError("导航地图PGM格式无效")
    width, height = (int(value) for value in dimensions.split())
    return values, resolution, origin, width, height, bytearray(raster)


def grid_location(point, resolution, origin, width, height):
    gx = int(math.floor((float(point["x"]) - float(origin[0])) / resolution))
    gy = int(math.floor((float(point["y"]) - float(origin[1])) / resolution))
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return None
    return gx, gy, height - 1 - gy


def point_is_navigable(point):
    yaml_path = latest_nav_map()
    if not yaml_path:
        raise ValueError("没有导航地图，请先保存三维地图")
    _, resolution, origin, width, height, raster = load_nav_grid(yaml_path)
    location = grid_location(point, resolution, origin, width, height)
    return bool(location and raster[location[2] * width + location[0]] >= 250)


def prepare_navigation_map(pose):
    yaml_path = latest_nav_map()
    if not yaml_path:
        raise ValueError("没有导航地图，请先保存三维地图")
    values, resolution, origin, width, height, raster = load_nav_grid(yaml_path)
    location = grid_location(pose, resolution, origin, width, height)
    if not location:
        raise ValueError("机器狗当前位置超出导航地图范围")
    gx, gy, row = location
    search = max(1, int(math.ceil(0.25 / resolution)))
    nearby_free = False
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            tx, ty = gx + dx, gy + dy
            if 0 <= tx < width and 0 <= ty < height:
                nearby_free |= raster[(height - 1 - ty) * width + tx] >= 250
    if not nearby_free:
        raise ValueError("当前位置附近没有可通行单元，请检查导航层或重新建图")
    # The robot physically occupies this footprint. Clear only its immediate
    # start circle; live MID360 obstacle data remains authoritative around it.
    radius = max(1, int(math.ceil(0.45 / resolution)))
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            tx, ty = gx + dx, gy + dy
            if 0 <= tx < width and 0 <= ty < height:
                raster[(height - 1 - ty) * width + tx] = 254
    pgm = RUNTIME / "navigation_map.pgm"
    yaml_runtime = RUNTIME / "navigation_map.yaml"
    pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + raster)
    yaml_runtime.write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {resolution:.6f}\n"
        f"origin: [{float(origin[0]):.6f}, {float(origin[1]):.6f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    return {"yaml": str(yaml_runtime), "source": str(yaml_path), "cleared_radius": 0.45}


def call_c12(path, payload):
    request = urllib.request.Request(
        C12_API + path,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        return json.load(response)


def execute_steps(run_id, task):
    steps = task.get("steps", [])
    checkpoints = {item["id"]: item for item in read_json(CHECKPOINTS_FILE, [])}
    try:
        for index, step in enumerate(steps):
            if TASK_CANCEL.is_set():
                raise InterruptedError("任务已停止")
            with LOCK:
                TASK_RUN.update(state="running", step=index, message=step.get("label", step.get("type", "")))
            kind = step.get("type")
            if kind == "goto":
                checkpoint = checkpoints.get(step.get("checkpoint_id"))
                if not checkpoint:
                    raise ValueError("任务引用的巡查点不存在")
                # The CMU planner stack exposes no Nav2-style action state, so
                # success is judged by the dog actually reaching the point:
                # robot pose (map_level, same frame as the checkpoint) within
                # arrive_radius for 4 consecutive samples, else the step times
                # out and motion is cut. Never report success on timeout.
                # Same hard gate as the manual goal endpoint (P0 audit #5):
                # LOCALIZED + map binding, or the task must not move the dog.
                loc = read_json(RUNTIME / "localization_state.json", {})
                active_mid = active_map_id()
                if time.time() - float(loc.get("updated_at", 0)) > 2.0 \
                        or loc.get("state") != "LOCALIZED" \
                        or not loc.get("ok_for_navigation"):
                    raise RuntimeError("全局定位未就绪（未LOCALIZED），任务无法导航")
                if loc.get("map_id") != active_mid:
                    raise RuntimeError("定位地图与活动地图不一致，任务无法导航")
                if checkpoint.get("map_name") not in (None, active_mid):
                    raise RuntimeError(f"巡查点属于地图{checkpoint.get('map_name')}，与活动地图不一致")
                goal = issue_goal(checkpoint)
                set_nav_motion(True, "任务导航：" + checkpoint["name"])
                deadline = time.time() + min(600, max(10, int(step.get("timeout", 120))))
                reach_radius = max(0.4, float(config().get("arrive_radius", 0.7)))
                near = 0
                while time.time() < deadline and not TASK_CANCEL.wait(0.5):
                    if not service_status().get("navigation", {}).get("running"):
                        set_nav_motion(False, "任务中断，运动已切断")
                        raise RuntimeError("导航服务已停止，任务中断")
                    pose = robot_state().get("pose", {})
                    distance = math.hypot(float(pose.get("x", 1e9)) - float(goal["x"]),
                                          float(pose.get("y", 1e9)) - float(goal["y"]))
                    near = near + 1 if distance <= reach_radius else 0
                    if near >= 4:
                        break
                if TASK_CANCEL.is_set():
                    raise InterruptedError("任务已停止")
                if near < 4:
                    set_nav_motion(False, "任务导航超时，运动已切断")
                    raise RuntimeError("导航超时：未能在时限内到达巡查点")
                set_nav_motion(False, "到达：" + checkpoint["name"])
            elif kind == "wait":
                if TASK_CANCEL.wait(max(0, min(3600, float(step.get("seconds", 1))))):
                    raise InterruptedError("任务已停止")
            elif kind == "palette":
                call_c12("/api/thermal/palette", {"code": step.get("code", "04")})
            elif kind == "preset":
                call_c12("/api/presets/call", {"id": step.get("preset_id", "")})
            elif kind == "photo":
                capture_c12(task["name"])
        with LOCK:
            TASK_RUN.update(state="completed", step=len(steps), message="任务完成")
    except InterruptedError as exc:
        with LOCK:
            TASK_RUN.update(state="canceled", message=str(exc))
    except Exception as exc:
        with LOCK:
            TASK_RUN.update(state="failed", message=str(exc))


def start_task(task):
    with LOCK:
        if TASK_RUN["state"] == "running":
            raise RuntimeError("已有任务正在执行")
        TASK_CANCEL.clear()
        run_id = uuid.uuid4().hex
        TASK_RUN.update(id=run_id, task_id=task["id"], state="running", step=0,
                        message="任务启动", started_at=time.strftime("%F %T"))
        threading.Thread(target=execute_steps, args=(run_id, task), daemon=True).start()
        return dict(TASK_RUN)


class Handler(BaseHTTPRequestHandler):
    server_version = "Go2Patrol/0.1"

    def log_message(self, fmt, *args):
        print(f"[{self.log_date_time_string()}] {fmt % args}", flush=True)

    def json_response(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path == "/api/status":
            self.json_response(200, {"config": config(), "robot": robot_state(), "c12": c12_status(),
                                     "go2": go2_state(),
                                     "services": service_status(), "task_run": dict(TASK_RUN),
                                     "teleop": dict(TELEOP), "mapping_session": mapping_session(), "localization": read_json(RUNTIME / "localization_state.json", {}),
                                     "mapping_recovery": read_json(MAPPING_RECOVERY_FILE, {})})
        elif path == "/api/checkpoints":
            self.json_response(200, read_json(CHECKPOINTS_FILE, []))
        elif path == "/api/tasks":
            self.json_response(200, read_json(TASKS_FILE, []))
        elif path == "/api/c12/presets":
            self.json_response(200, c12_presets())
        elif path == "/api/map":
            self.json_response(200, read_json(MAP_FILE, {"available": False, "width": 0, "height": 0}))
        elif path == "/api/cloud":
            self.json_response(200, read_json(CLOUD_FILE, {"available": False, "total_points": 0, "shown_points": 0, "points": []}))
        elif path == "/api/navigation/path":
            route = read_json(NAV_PATH_FILE, {"available": False, "points": []})
            age = time.time() - float(route.get("updated_at", 0))
            if age > 3.0 or not service_status().get("navigation", {}).get("running"):
                route = {"available": False, "points": [], "updated_at": route.get("updated_at")}
            self.json_response(200, route)
        elif path == "/api/maps":
            inventory = map_inventory()
            self.json_response(200, {"maps": inventory, "active": next((item["name"] for item in inventory if item["active"]), None)})
        elif path == "/api/maps/cloud":
            name = parse_qs(urlparse(self.path).query).get("name", [""])[0]
            self.json_response(200, load_pcd_cloud(name))
        elif path.startswith("/api/logs/"):
            name = Path(path).name
            if name not in ("mapping", "navigation"):
                self.send_error(404); return
            log = RUNTIME / "logs" / f"{name}.log"
            text = log.read_text(encoding="utf-8", errors="replace")[-12000:] if log.exists() else ""
            self.json_response(200, {"log": text})
        elif path == "/" or path.startswith("/static/"):
            relative = "index.html" if path == "/" else path[len("/static/"):]
            target = (STATIC / relative).resolve()
            if STATIC.resolve() not in target.parents and target != STATIC.resolve():
                self.send_error(403); return
            if not target.is_file():
                self.send_error(404); return
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers(); self.wfile.write(data)
        else:
            self.send_error(404)

    def do_POST(self):
        path = unquote(urlparse(self.path).path)
        try:
            data = self.body()
            if path == "/api/config":
                new = config()
                for key in DEFAULT_CONFIG:
                    if key in data: new[key] = str(data[key])
                if new["sensor"] not in ("go2", "mid360"): raise ValueError("雷达类型无效")
                write_json(CONFIG_FILE, new); self.json_response(200, new)
            elif path == "/api/mapping/start":
                current = config(); current.update({"sensor": "mid360", "odom_topic": "/Odometry", "map_frame": "map_level"}); write_json(CONFIG_FILE, current)
                teleop_disarm("建图启动中，控制已锁定")
                stop_service("navigation")
                if service_status().get("mapping", {}).get("running"):
                    self.json_response(200, service_status()["mapping"]); return
                session = {"id": uuid.uuid4().hex, "started_at": time.time(), "started_text": time.strftime("%F %T")}
                write_json(MAPPING_SESSION_FILE, session); CLOUD_FILE.unlink(missing_ok=True)
                result = start_service("mapping", "run_mid360_nav_mapping_stack.sh")
                result["session_id"] = session["id"]
                self.json_response(200, result)
            elif path == "/api/mapping/stop":
                teleop_disarm("建图停止，控制已锁定")
                autosave = None; save_error = None
                state = robot_state()
                if state.get("online") and state.get("cloud_map_online"):
                    try: autosave = save_cloud_map("autosave_stop_" + time.strftime("%Y%m%d_%H%M%S"))
                    except Exception as exc: save_error = str(exc)
                stopped = stop_service("mapping")
                self.json_response(200, {"stopped": stopped, "autosave": autosave, "save_error": save_error})
            elif path == "/api/radar/stop":
                self.json_response(200, stop_radar_data())
            elif path == "/api/robot/action":
                actions = {"stand_up": (1004, "站立"), "stand_down": (1005, "趴下")}
                action = str(data.get("action", ""))
                if action not in actions: raise ValueError("不支持的姿态动作")
                if data.get("confirm") is not True: raise ValueError("必须确认现场安全后才能执行姿态动作")
                if not go2_state().get("online"): raise ValueError("GO2状态离线，不能执行姿态动作")
                if action == "stand_up": require_go2_motion_safe(require_standing=False)
                teleop_disarm("执行" + actions[action][1] + "，控制已锁定")
                command = {"id": uuid.uuid4().hex, "api_id": actions[action][0], "action": action,
                           "created_at": time.time()}
                write_json(GO2_ACTION_FILE, command)
                self.json_response(200, command)
            elif path == "/api/teleop/arm":
                # Compatibility endpoint for older cached pages. Manual control
                # no longer has a separate arm/unlock state.
                teleop_command(force_stop=True)
                TELEOP.update(armed=True, active_key=None, message="直接控制 · 按住运动，松开停止")
                self.json_response(200, dict(TELEOP))
            elif path == "/api/teleop":
                key = str(data.get("key", "k")).lower()
                pressed = bool(data.get("pressed", False))
                if key == "k" or not pressed:
                    self.json_response(200, teleop_command(force_stop=True))
                else:
                    if service_status().get("navigation", {}).get("running"):
                        raise ValueError("自主导航运行中，不能同时使用手动控制")
                    if not go2_state(max_age=6.0).get("online"):
                        teleop_disarm("机器狗状态离线，已停止")
                        raise ValueError("机器狗状态离线，控制已停止")
                    require_go2_motion_safe(max_age=6.0)
                    self.json_response(200, teleop_command(key, data.get("speed", 0.10)))
            elif path == "/api/maps/save":
                if not service_status().get("mapping", {}).get("running"):
                    raise ValueError("建图会话已结束，停止时已自动保存，无需再次保存")
                result = save_cloud_map(str(data.get("name", ""))) if config()["sensor"] == "mid360" else save_map(str(data.get("name", "")))
                # Activate only a localization-ready package (P1 audit: a
                # map whose DB failed must never become the navigation map).
                if result.get("name") and result.get("localization_ready") and not result.get("nav_error"):
                    write_json(ACTIVE_MAP_FILE, {"name": result["name"], "updated_at": time.time()})
                self.json_response(200, result)
            elif path == "/api/maps/activate":
                if service_status().get("navigation", {}).get("running"):
                    raise ValueError("请先停止导航，再切换活动地图")
                self.json_response(200, select_map(data.get("name", "")))
            elif path == "/api/maps/continue":
                name = Path(str(data.get("name", ""))).stem
                current = robot_state()
                newest = map_inventory()[0]["name"] if map_inventory() else None
                if current.get("cloud_map_online") and name == newest:
                    self.json_response(200, {"ok": True, "mode": "live", "message": "当前FAST-LIO会话仍在线，可继续移动并累计，完成后请另存新版本。"})
                else:
                    raise ValueError("当前FAST-LIO版本不能从已保存PCD恢复滤波器状态；跨重启续建暂不可用。可查看和导航此地图，真正续建需后续接入地图加载/重定位模块。")
            elif path == "/api/maps/delete":
                name = Path(str(data.get("name", ""))).stem
                if not name:
                    raise ValueError("缺少地图名称")
                target = DATA / "maps" / (name + ".pcd")
                package_dir = DATA / "maps" / name
                if not target.is_file() and not package_dir.is_dir():
                    raise ValueError("三维地图不存在")
                if service_status().get("navigation", {}).get("running") and \
                        read_json(ACTIVE_MAP_FILE, {}).get("name") == name:
                    raise ValueError("该地图正在用于导航，请先停止导航再删除")
                active_meta = read_json(target.with_suffix(".meta.json"), {})
                if service_status().get("mapping", {}).get("running") and \
                        active_meta.get("session_id") == read_json(MAPPING_SESSION_FILE, {}).get("id"):
                    raise ValueError("该地图对应正在进行的建图会话，请先停止建图再删除")
                removed_files = 0
                package_dir = DATA / "maps" / name
                if package_dir.is_dir():
                    shutil.rmtree(package_dir)
                    removed_files += 1
                for suffix in (".pcd", ".npy", ".meta.json", ".pgm", ".yaml", ".png"):
                    candidate = DATA / "maps" / (name + suffix)
                    if candidate.is_file():
                        candidate.unlink()
                        removed_files += 1
                if read_json(ACTIVE_MAP_FILE, {}).get("name") == name:
                    write_json(ACTIVE_MAP_FILE, {})
                points = read_json(CHECKPOINTS_FILE, [])
                kept = [p for p in points if p.get("map_name") != name]
                removed_points = len(points) - len(kept)
                if removed_points:
                    write_json(CHECKPOINTS_FILE, kept)
                self.json_response(200, {"ok": True, "removed_files": removed_files,
                                         "removed_checkpoints": removed_points})
            elif path == "/api/navigation/start":
                # Dual-mode perception: mapping owns the lidar with
                # map_en=true; navigation needs localization mode with the
                # global localizer. They are mutually exclusive. Reachability
                # checks belong to the GOAL layer (after LOCALIZED), not here:
                # before localization succeeds the dog has no valid
                # map_level pose at all.
                if service_status().get("mapping", {}).get("running"):
                    raise ValueError("三维建图运行中，请先停止建图再启动导航")
                teleop_disarm("自主导航启动，手动控制已锁定")
                set_nav_motion(False, "导航服务启动，等待定位与用户确认目标")
                GOAL_FILE.unlink(missing_ok=True)
                start_service("navigation", "run_navigation_stack_v2.sh")
                self.json_response(200, require_service_stays_running("navigation", 4.0))
            elif path == "/api/navigation/stop":
                set_nav_motion(False, "用户停止自主导航")
                teleop_disarm("自主导航停止，控制已锁定")
                GOAL_FILE.unlink(missing_ok=True)
                self.json_response(200, {"stopped": stop_service("navigation")})
            elif path == "/api/checkpoints/save":
                name = str(data.get("name", "")).strip()[:40]
                if not name: raise ValueError("请输入巡查点名称")
                points = read_json(CHECKPOINTS_FILE, [])
                point = {"id": uuid.uuid4().hex, "name": name, "x": float(data["x"]), "y": float(data["y"]),
                         "z": float(data.get("z", 0)),
                         "yaw": float(data.get("yaw", 0)), "created_at": time.strftime("%F %T")}
                session = mapping_session()
                if robot_state().get("online"):
                    # Tag every point picked while live localization is online:
                    # session id identifies the FAST-LIO frame the coordinates
                    # are valid in; __live__ marks it as this session's draft.
                    # (The old mapping-running requirement left points picked
                    # between sessions untagged and invisible everywhere.)
                    loc = read_json(RUNTIME / "localization_state.json", {})
                    active_mid = active_map_id()
                    loc_ready = (loc.get("state") == "LOCALIZED"
                                 and loc.get("ok_for_navigation")
                                 and loc.get("map_id") == active_mid
                                 and time.time() - float(loc.get("updated_at", 0)) <= 2.0)
                    if loc_ready:
                        # Navigation mode: bind straight to the LOCALIZED map
                        # so a point can never be navigated with another
                        # map's coordinates.
                        point["map_name"] = active_mid
                        point["session_id"] = session.get("id")
                    elif session.get("running") and active_mid:
                        # Mapping mode: draft bound to the live session.
                        point["map_name"] = "__live__"
                        point["session_id"] = session.get("id")
                points.append(point); write_json(CHECKPOINTS_FILE, points); self.json_response(200, point)
            elif path == "/api/checkpoints/delete":
                points = read_json(CHECKPOINTS_FILE, [])
                updated = [p for p in points if p["id"] != str(data.get("id"))]
                write_json(CHECKPOINTS_FILE, updated); self.json_response(200, {"ok": len(updated) != len(points)})
            elif path == "/api/navigation/goal":
                nav_service = service_status().get("navigation", {})
                if not nav_service.get("running"):
                    raise ValueError("导航服务未启动，请先点击“启动导航”并等待状态就绪")
                try:
                    nav_started = time.mktime(time.strptime(nav_service.get("started_at", ""), "%Y-%m-%d %H:%M:%S"))
                except (TypeError, ValueError):
                    nav_started = time.time()
                if time.time() - nav_started < 7.0:
                    raise ValueError("导航控制器正在激活，请等待约7秒后再下发目标")
                if not robot_state().get("online") or not robot_state().get("cloud_map_online"):
                    raise ValueError("实时定位或MID360点云已中断，禁止下发导航目标")
                if data.get("confirm") is not True:
                    raise ValueError("必须确认周围安全后才能下发导航目标")
                require_go2_motion_safe()
                point = next((p for p in read_json(CHECKPOINTS_FILE, []) if p["id"] == str(data.get("id"))), None)
                if not point: raise ValueError("巡查点不存在")
                map_mode = False
                active_mid = active_map_id()
                if active_mid:
                    packaged_meta = read_json(DATA / "maps" / active_mid / "metadata.json", {})
                    map_mode = bool(packaged_meta.get("localization_ready"))
                active_name = active_mid if map_mode else None
                session_ok = (point.get("map_name") == "__live__" and
                              point.get("session_id") == mapping_session().get("id") and
                              service_status().get("mapping", {}).get("running"))
                if point.get("map_name") != active_name and not session_ok:
                    raise ValueError("该巡查点不属于当前活动地图或本次实时会话，请重新选择")
                current_pose = robot_state().get("pose", {})
                if active_name:
                    # 2-D grid checks only exist for a saved navigation map;
                    # live-session navigation relies on the planner's own
                    # terrain corridor instead.
                    if not point_is_navigable(current_pose):
                        speak_navigation("导航失败")
                        raise ValueError("机器狗当前位置不在当前地图的可通行区域，禁止规划")
                    if not point_is_navigable(point):
                        raise ValueError("该点不在导航地图的可通行区域，请重新选择地面位置")
                # Final-plan gate layer: the global localizer must be
                # LOCALIZED for the active map before any goal is issued.
                loc = read_json(RUNTIME / "localization_state.json", {})
                if time.time() - float(loc.get("updated_at", 0)) > 2.0 \
                        or loc.get("state") != "LOCALIZED" \
                        or not loc.get("ok_for_navigation"):
                    raise ValueError("全局定位未就绪（未LOCALIZED），禁止下发目标")
                if active_mid and loc.get("map_id") != active_mid:
                    raise ValueError("定位所在的地图与活动地图不一致，禁止下发目标")
                baseline = int(go2_state().get("nav_relay", {}).get("publish_count", 0) or 0)
                set_nav_motion(True, "导航到：" + point["name"])
                goal = issue_goal(point)
                threading.Thread(target=monitor_navigation_announcement,
                                 args=(goal["id"], baseline), daemon=True).start()
                self.json_response(200, goal)
            elif path == "/api/tasks/save":
                name = str(data.get("name", "")).strip()[:60]
                if not name: raise ValueError("请输入任务名称")
                tasks = read_json(TASKS_FILE, [])
                task_id = str(data.get("id") or uuid.uuid4().hex)
                task = {"id": task_id, "name": name, "workspace": data.get("workspace", {}),
                        "steps": data.get("steps", []), "updated_at": time.strftime("%F %T")}
                tasks = [t for t in tasks if t["id"] != task_id] + [task]
                write_json(TASKS_FILE, tasks); self.json_response(200, task)
            elif path == "/api/tasks/delete":
                tasks = read_json(TASKS_FILE, [])
                updated = [t for t in tasks if t["id"] != str(data.get("id"))]
                write_json(TASKS_FILE, updated); self.json_response(200, {"ok": len(updated) != len(tasks)})
            elif path == "/api/tasks/run":
                if not service_status().get("navigation", {}).get("running"):
                    raise ValueError("导航服务未启动，请先启动导航后再运行巡逻任务")
                if data.get("confirm") is not True:
                    raise ValueError("必须确认现场安全后才能运行巡逻任务")
                require_go2_motion_safe()
                task = next((t for t in read_json(TASKS_FILE, []) if t["id"] == str(data.get("id"))), None)
                if not task: raise ValueError("任务不存在")
                for step in task.get("steps", []):
                    if step.get("type") == "goto":
                        point = next((p for p in read_json(CHECKPOINTS_FILE, []) if p["id"] == step.get("checkpoint_id")), None)
                        if not point or not point_is_navigable(point):
                            raise ValueError("任务包含无效或不可通行的巡查点")
                set_nav_motion(True, "运行巡逻任务：" + task["name"])
                self.json_response(200, start_task(task))
            elif path == "/api/tasks/stop":
                set_nav_motion(False, "用户停止巡逻任务")
                TASK_CANCEL.set(); self.json_response(200, {"ok": True})
            else:
                self.send_error(404)
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            self.json_response(400, {"error": str(exc)})


def shutdown(*_):
    TASK_CANCEL.set()
    # Wi-Fi and the web console are only supervision channels. Mapping,
    # localization and autonomous navigation must survive a web restart.
    # Manual teleoperation remains fail-safe through its short command lease.
    teleop_command(force_stop=True)
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)
    try: start_service("go2_state", "run_go2_state_bridge.sh")
    except Exception as exc: print("GO2 state bridge start failed:", exc, flush=True)
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer((HOST, PORT), Handler); server.daemon_threads = True
    print(f"Patrol console: http://{HOST}:{PORT}", flush=True)
    server.serve_forever()
