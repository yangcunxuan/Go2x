#!/usr/bin/env python3
"""Go2 patrol console: device status, maps, checkpoints and block tasks."""

import json
import math
import mimetypes
import os
import signal
import subprocess
import threading
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

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
        log = open(log_dir / f"{name}.log", "ab", buffering=0)
        process = subprocess.Popen(
            ["bash", str(PROJECT / "scripts" / script)],
            cwd=PROJECT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        SERVICES[name] = {"process": process, "started_at": time.strftime("%F %T"), "log": log}
        return {"running": True, "pid": process.pid, "started_at": SERVICES[name]["started_at"]}


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
    state["online"] = bool(state["bridge_online"] and state.get("odom_online", False))
    state.setdefault("pose", {"x": 0, "y": 0, "yaw": 0})
    state.setdefault("topics", [])
    return state


def issue_goal(checkpoint):
    goal = {
        "id": uuid.uuid4().hex,
        "type": "navigate",
        "x": checkpoint["x"], "y": checkpoint["y"],
        "yaw": checkpoint.get("yaw", 0),
        "checkpoint_id": checkpoint["id"],
        "created_at": time.time(),
    }
    write_json(GOAL_FILE, goal)
    return goal


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
                                     "services": service_status(), "task_run": dict(TASK_RUN),
                                     "teleop": dict(TELEOP)})
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
                CLOUD_FILE.unlink(missing_ok=True)
                self.json_response(200, start_service("mapping", "run_mid360_nav_mapping_stack.sh"))
            elif path == "/api/mapping/stop":
                teleop_disarm("建图停止，控制已锁定")
                self.json_response(200, {"stopped": stop_service("mapping")})
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
                self.json_response(200, save_cloud_map(str(data.get("name", ""))) if config()["sensor"] == "mid360" else save_map(str(data.get("name", ""))))
            elif path == "/api/navigation/start":
                if not list((DATA / "maps").glob("*.yaml")):
                    raise ValueError("没有已保存地图，请先完成建图并点击“保存地图”")
                teleop_disarm("自主导航启动，手动控制已锁定")
                stop_service("mapping")
                # A target clicked while only mapping must never be executed
                # automatically by a newly started navigation process.
                GOAL_FILE.unlink(missing_ok=True)
                self.json_response(200, start_service("navigation", "run_navigation_stack.sh"))
            elif path == "/api/navigation/stop":
                teleop_disarm("自主导航停止，控制已锁定")
                self.json_response(200, {"stopped": stop_service("navigation")})
            elif path == "/api/checkpoints/save":
                name = str(data.get("name", "")).strip()[:40]
                if not name: raise ValueError("请输入巡查点名称")
                points = read_json(CHECKPOINTS_FILE, [])
                point = {"id": uuid.uuid4().hex, "name": name, "x": float(data["x"]), "y": float(data["y"]),
                         "yaw": float(data.get("yaw", 0)), "created_at": time.strftime("%F %T")}
                points.append(point); write_json(CHECKPOINTS_FILE, points); self.json_response(200, point)
            elif path == "/api/checkpoints/delete":
                points = read_json(CHECKPOINTS_FILE, [])
                updated = [p for p in points if p["id"] != str(data.get("id"))]
                write_json(CHECKPOINTS_FILE, updated); self.json_response(200, {"ok": len(updated) != len(points)})
            elif path == "/api/navigation/goal":
                if not service_status().get("navigation", {}).get("running"):
                    raise ValueError("导航服务未启动，请先点击“启动导航”并等待状态就绪")
                point = next((p for p in read_json(CHECKPOINTS_FILE, []) if p["id"] == str(data.get("id"))), None)
                if not point: raise ValueError("巡查点不存在")
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
                task = next((t for t in read_json(TASKS_FILE, []) if t["id"] == str(data.get("id"))), None)
                if not task: raise ValueError("任务不存在")
                self.json_response(200, start_task(task))
            elif path == "/api/tasks/stop":
                TASK_CANCEL.set(); self.json_response(200, {"ok": True})
            else:
                self.send_error(404)
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            self.json_response(400, {"error": str(exc)})


def shutdown(*_):
    TASK_CANCEL.set()
    teleop_disarm("网页服务停止，控制已锁定")
    for name in list(SERVICES): stop_service(name)
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)
    server = ThreadingHTTPServer((HOST, PORT), Handler); server.daemon_threads = True
    print(f"Patrol console: http://{HOST}:{PORT}", flush=True)
    server.serve_forever()
