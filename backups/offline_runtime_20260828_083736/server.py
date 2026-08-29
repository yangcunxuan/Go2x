#!/usr/bin/env python3
"""Go2 patrol console: device status, maps, checkpoints and block tasks."""

import json
import math
import mimetypes
import os
import signal
import struct
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
CLOUD_FILE = RUNTIME / "cloud.json"
CLOUD_SAVE_REQUEST = RUNTIME / "cloud_save_request.json"
CLOUD_SAVE_RESPONSE = RUNTIME / "cloud_save_response.json"
GOAL_FILE = RUNTIME / "goal.json"
TELEOP_FILE = RUNTIME / "teleop.json"
NAV_ENABLE_FILE = RUNTIME / "nav_motion_enable.json"
GO2_STATE_FILE = RUNTIME / "go2_state.json"
GO2_ACTION_FILE = RUNTIME / "go2_action.json"
ACTIVE_MAP_FILE = DATA / "active_map.json"
MAPPING_SESSION_FILE = RUNTIME / "mapping_session.json"
MAPPING_RECOVERY_FILE = RUNTIME / "mapping_recovery.json"

DEFAULT_CONFIG = {
    "sensor": "mid360",
    "robot_name": "GO2X",
    "scan_topic": "/scan",
    "odom_topic": "/Odometry",
    "cmd_vel_topic": "/cmd_vel",
    "map_frame": "map_level",
    "base_frame": "base_link",
}

LOCK = threading.RLock()
SERVICES = {}
TASK_RUN = {"id": None, "task_id": None, "state": "idle", "step": 0, "message": "", "started_at": None}
TASK_CANCEL = threading.Event()
TELEOP = {"armed": False, "active_key": None, "updated_at": 0.0, "message": "控制已锁定"}
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
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, path)


def teleop_command(key="k", speed=0.0, force_stop=False):
    """Write one short-lived velocity command consumed by patrol_bridge."""
    key = str(key).lower()
    if key not in TELEOP_DIRECTIONS:
        raise ValueError("不支持的控制按键")
    speed = max(0.05, min(0.30, float(speed))) if not force_stop else 0.0
    x, y, yaw = TELEOP_DIRECTIONS[key]
    if key in ("u", "o", "m", "."):
        x *= 0.7071; y *= 0.7071
    command = {
        "id": uuid.uuid4().hex, "updated_at": time.time(),
        "key": "k" if force_stop else key,
        "vx": 0.0 if force_stop else x * speed,
        "vy": 0.0 if force_stop else y * speed,
        "vyaw": 0.0 if force_stop else yaw * min(0.50, speed * 2.0),
    }
    write_json(TELEOP_FILE, command)
    TELEOP.update(active_key=None if force_stop or key == "k" else key,
                  updated_at=command["updated_at"],
                  message="已停止" if force_stop or key == "k" else f"按住 {key.upper()} 运动")
    return command


def teleop_disarm(message="控制已锁定"):
    teleop_command(force_stop=True)
    TELEOP.update(armed=False, active_key=None, message=message)


def config():
    result = dict(DEFAULT_CONFIG)
    result.update(read_json(CONFIG_FILE, {}))
    return result


def service_status():
    result = {}
    with LOCK:
        for name, item in list(SERVICES.items()):
            process = item["process"]
            result[name] = {
                "running": process.poll() is None,
                "pid": process.pid,
                "started_at": item["started_at"],
                "returncode": process.poll(),
            }
    return result


def start_service(name, script):
    with LOCK:
        existing = SERVICES.get(name)
        if existing and existing["process"].poll() is None:
            return service_status()[name]
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
        return {"running": True, "pid": process.pid, "started_at": SERVICES[name]["started_at"]}


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
        if not item or item["process"].poll() is not None:
            return False
        try:
            os.killpg(item["process"].pid, signal.SIGINT)
            item["process"].wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(item["process"].pid, signal.SIGTERM)
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
    request_id = uuid.uuid4().hex
    write_json(CLOUD_SAVE_REQUEST, {"id": request_id, "name": safe, "created_at": time.time()})
    deadline = time.time() + 15
    while time.time() < deadline:
        response = read_json(CLOUD_SAVE_RESPONSE, {})
        if response.get("id") == request_id:
            if not response.get("ok"):
                raise RuntimeError(response.get("error", "三维地图保存失败"))
            command = ["docker-compose", "run", "--rm", "ros2", "python3",
                       "/project/scripts/pcd_to_nav_map.py", response["pcd"]]
            projection = subprocess.run(command, cwd=PROJECT, capture_output=True, text=True, timeout=60)
            if projection.returncode:
                response["nav_error"] = (projection.stderr or projection.stdout or "导航层生成失败")[-500:]
            else:
                response["nav_map"] = json.loads(projection.stdout.strip().splitlines()[-1])
            session = read_json(MAPPING_SESSION_FILE, {})
            meta = {"name": safe, "session_id": session.get("id"), "session_started_at": session.get("started_at"),
                    "saved_at": time.time(), "coordinate_frame": "map_level"}
            write_json(DATA / "maps" / f"{safe}.meta.json", meta)
            points = read_json(CHECKPOINTS_FILE, [])
            changed = False
            for point in points:
                if session.get("id") and point.get("session_id") == session.get("id"):
                    point["map_name"] = safe; changed = True
            if changed: write_json(CHECKPOINTS_FILE, points)
            write_json(ACTIVE_MAP_FILE, {"name": safe, "updated_at": time.time()})
            response["session_id"] = session.get("id")
            return response
        time.sleep(0.15)
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
        state["cloud_map_online"] = False
    state["online"] = bool(state["bridge_online"] and state.get("odom_online", False))
    state.setdefault("pose", {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0})
    state.setdefault("topics", [])
    return state


def mapping_session():
    session = read_json(MAPPING_SESSION_FILE, {})
    session["running"] = bool(service_status().get("mapping", {}).get("running"))
    return session


def go2_state():
    state = read_json(GO2_STATE_FILE, {})
    updated = float(state.get("updated_at", 0))
    state["online"] = bool(updated and time.time() - updated < 3 and
                           (state.get("lowstate_online") or state.get("sportstate_online")))
    state["age"] = round(time.time() - updated, 2) if updated else None
    return state


def issue_goal(checkpoint):
    goal = {
        "id": uuid.uuid4().hex,
        "type": "navigate",
        "x": checkpoint["x"], "y": checkpoint["y"], "z": checkpoint.get("z", 0),
        "yaw": checkpoint.get("yaw", 0),
        "checkpoint_id": checkpoint["id"],
        "created_at": time.time(),
    }
    write_json(GOAL_FILE, goal)
    return goal


def set_nav_motion(enabled, reason=""):
    state = {
        "enabled": bool(enabled), "reason": str(reason), "updated_at": time.time(),
        "expires_at": time.time() + 600 if enabled else 0,
    }
    write_json(NAV_ENABLE_FILE, state)
    return state


def latest_nav_map():
    active = read_json(ACTIVE_MAP_FILE, {}).get("name")
    if active:
        selected = DATA / "maps" / (Path(active).stem + ".yaml")
        if selected.is_file():
            return selected
    maps = sorted((DATA / "maps").glob("*.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)
    return maps[0] if maps else None


def map_inventory():
    active = read_json(ACTIVE_MAP_FILE, {}).get("name")
    if not active:
        candidates = sorted((DATA / "maps").glob("*.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)
        active = candidates[0].stem if candidates else None
    result = []
    for pcd in sorted((DATA / "maps").glob("*.pcd"), key=lambda p: p.stat().st_mtime, reverse=True):
        points = None
        try:
            with open(pcd, "rb") as handle:
                for _ in range(40):
                    line = handle.readline().decode("ascii", "ignore").strip()
                    if line.startswith("POINTS "): points = int(line.split()[1])
                    if line.startswith("DATA "): break
        except (OSError, ValueError):
            pass
        meta = read_json(pcd.with_suffix(".meta.json"), {})
        current_session = read_json(MAPPING_SESSION_FILE, {}).get("id")
        result.append({"name": pcd.stem, "pcd": pcd.name, "points": points,
                       "bytes": pcd.stat().st_size, "modified_at": pcd.stat().st_mtime,
                       "nav_ready": pcd.with_suffix(".yaml").is_file(), "active": pcd.stem == active,
                       "session_id": meta.get("session_id"),
                       "current_session": bool(current_session and meta.get("session_id") == current_session)})
    return result


def select_map(name):
    safe = Path(str(name)).stem
    target = DATA / "maps" / (safe + ".pcd")
    if not target.is_file() or target.parent != (DATA / "maps"):
        raise ValueError("三维地图不存在")
    if not target.with_suffix(".yaml").is_file():
        raise ValueError("该PCD尚未生成导航层")
    write_json(ACTIVE_MAP_FILE, {"name": safe, "updated_at": time.time()})
    return {"ok": True, "name": safe}


def load_pcd_cloud(name):
    safe = Path(str(name)).stem
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
                issue_goal(checkpoint)
                # The ROS bridge updates nav_status. Offline mode does not block forever.
                deadline = time.time() + min(600, max(10, int(step.get("timeout", 120))))
                saw_active = False
                while time.time() < deadline and not TASK_CANCEL.wait(0.5):
                    nav = robot_state().get("nav_status", "offline")
                    saw_active = saw_active or nav in ("accepted", "navigating")
                    if saw_active and nav == "succeeded":
                        break
                    if saw_active and nav in ("failed", "canceled"):
                        raise RuntimeError("导航未成功：" + nav)
                    if not robot_state()["online"]:
                        break
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
                                     "teleop": dict(TELEOP), "mapping_session": mapping_session(),
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
                teleop_disarm("执行" + actions[action][1] + "，控制已锁定")
                command = {"id": uuid.uuid4().hex, "api_id": actions[action][0], "action": action,
                           "created_at": time.time()}
                write_json(GO2_ACTION_FILE, command)
                self.json_response(200, command)
            elif path == "/api/teleop/arm":
                armed = bool(data.get("armed", False))
                if armed:
                    if service_status().get("navigation", {}).get("running"):
                        raise ValueError("自主导航运行中，不能同时启用手动控制")
                    if not robot_state().get("online"):
                        raise ValueError("机器狗 ROS 尚未在线，请先开始建图并等待状态变绿")
                    teleop_command(force_stop=True)
                    TELEOP.update(armed=True, active_key=None, message="控制已解锁，按住方向键运动")
                else:
                    teleop_disarm()
                self.json_response(200, dict(TELEOP))
            elif path == "/api/teleop":
                key = str(data.get("key", "k")).lower()
                pressed = bool(data.get("pressed", False))
                if key == "k" or not pressed:
                    self.json_response(200, teleop_command(force_stop=True))
                else:
                    if not TELEOP["armed"]:
                        raise ValueError("请先解锁机器狗控制")
                    if not robot_state().get("online"):
                        teleop_disarm("机器狗状态离线，已自动锁定")
                        raise ValueError("机器狗状态离线，控制已停止")
                    self.json_response(200, teleop_command(key, data.get("speed", 0.10)))
            elif path == "/api/maps/save":
                result = save_cloud_map(str(data.get("name", ""))) if config()["sensor"] == "mid360" else save_map(str(data.get("name", "")))
                if result.get("name") and not result.get("nav_error"):
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
            elif path == "/api/navigation/start":
                state = robot_state()
                if not service_status().get("mapping", {}).get("running") or not state.get("online") or not state.get("cloud_map_online"):
                    raise ValueError("FAST-LIO实时定位未在线。导航必须保持本次MID360建图运行，不能停止建图后再启动导航。")
                session = mapping_session()
                active = latest_nav_map()
                active_meta = read_json(active.with_suffix(".meta.json"), {}) if active else {}
                if not active or active_meta.get("session_id") != session.get("id"):
                    save_cloud_map("nav_auto_" + time.strftime("%Y%m%d_%H%M%S"))
                prepare_navigation_map(state["pose"])
                teleop_disarm("自主导航启动，手动控制已锁定")
                set_nav_motion(False, "导航服务启动，等待用户确认目标")
                # A target clicked while only mapping must never be executed
                # automatically by a newly started navigation process.
                GOAL_FILE.unlink(missing_ok=True)
                start_service("navigation", "run_navigation_stack.sh")
                self.json_response(200, require_service_stays_running("navigation", 4.0))
            elif path == "/api/navigation/stop":
                set_nav_motion(False, "用户停止自主导航")
                teleop_disarm("自主导航停止，控制已锁定")
                self.json_response(200, {"stopped": stop_service("navigation")})
            elif path == "/api/checkpoints/save":
                name = str(data.get("name", "")).strip()[:40]
                if not name: raise ValueError("请输入巡查点名称")
                points = read_json(CHECKPOINTS_FILE, [])
                point = {"id": uuid.uuid4().hex, "name": name, "x": float(data["x"]), "y": float(data["y"]),
                         "z": float(data.get("z", 0)),
                         "yaw": float(data.get("yaw", 0)), "created_at": time.strftime("%F %T")}
                session = mapping_session()
                if session.get("running") and robot_state().get("online"):
                    point["session_id"] = session.get("id"); point["map_name"] = "__live__"
                points.append(point); write_json(CHECKPOINTS_FILE, points); self.json_response(200, point)
            elif path == "/api/checkpoints/delete":
                points = read_json(CHECKPOINTS_FILE, [])
                updated = [p for p in points if p["id"] != str(data.get("id"))]
                write_json(CHECKPOINTS_FILE, updated); self.json_response(200, {"ok": len(updated) != len(points)})
            elif path == "/api/navigation/goal":
                if not service_status().get("navigation", {}).get("running"):
                    raise ValueError("导航服务未启动，请先点击“启动导航”并等待状态就绪")
                if not robot_state().get("online") or not robot_state().get("cloud_map_online"):
                    raise ValueError("实时定位或MID360点云已中断，禁止下发导航目标")
                if data.get("confirm") is not True:
                    raise ValueError("必须确认周围安全后才能下发导航目标")
                point = next((p for p in read_json(CHECKPOINTS_FILE, []) if p["id"] == str(data.get("id"))), None)
                if not point: raise ValueError("巡查点不存在")
                active_name = latest_nav_map().stem if latest_nav_map() else None
                if point.get("map_name") != active_name:
                    raise ValueError("该巡查点不属于当前活动地图，请在本次建图中重新创建或选用对应地图")
                if not point_is_navigable(point):
                    raise ValueError("该点不在导航地图的可通行区域，请重新选择地面位置")
                set_nav_motion(True, "导航到：" + point["name"])
                self.json_response(200, issue_goal(point))
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
    set_nav_motion(False, "网页服务停止")
    teleop_disarm("网页服务停止，控制已锁定")
    for name in list(SERVICES): stop_service(name)
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)
    try: start_service("go2_state", "run_go2_state_bridge.sh")
    except Exception as exc: print("GO2 state bridge start failed:", exc, flush=True)
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer((HOST, PORT), Handler); server.daemon_threads = True
    print(f"Patrol console: http://{HOST}:{PORT}", flush=True)
    server.serve_forever()
