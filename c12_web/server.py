#!/usr/bin/env python3
"""C12 dual-spectrum preview and gimbal control, dependency-free except FFmpeg."""

import json
import os
import signal
import socket
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

CAMERA_IP = os.environ.get("C12_IP", "192.168.144.108")
CONTROL_PORT = int(os.environ.get("C12_CONTROL_PORT", "5000"))
HTTP_HOST = os.environ.get("C12_HTTP_HOST", "127.0.0.1")
HTTP_PORT = int(os.environ.get("C12_HTTP_PORT", "8088"))
STEP_DURATION = float(os.environ.get("C12_STEP_DURATION", "0.16"))
STEP_SPEED = int(os.environ.get("C12_STEP_SPEED", "20"))
DEFAULT_HOLD_SPEED = int(os.environ.get("C12_HOLD_SPEED", "36"))
PRESETS_FILE = os.environ.get(
    "C12_PRESETS_FILE", os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets.json")
)

STREAMS = {
    "visible": f"rtsp://{CAMERA_IP}:554/stream=1",
    "thermal": f"rtsp://{CAMERA_IP}:555/stream=2",
}

PTZ_CENTER = "#TPUG2wPTZ05"
DIRECTIONS = {
    "up": (0.0, 1.0),
    "down": (0.0, -1.0),
    "left": (-1.0, 0.0),
    "right": (1.0, 0.0),
    "up_left": (-0.707, 0.707),
    "up_right": (0.707, 0.707),
    "down_left": (-0.707, -0.707),
    "down_right": (0.707, -0.707),
}
ACTIONS = set(DIRECTIONS) | {"stop", "center"}

CONTROL_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CONTROL_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
CONTROL_SOCKET.bind(("0.0.0.0", CONTROL_PORT))
CONTROL_SOCKET.settimeout(0.5)
CONTROL_LOCK = threading.Lock()
ATTITUDE_LOCK = threading.Lock()
ATTITUDE_STOP = threading.Event()
ATTITUDE = {"yaw": None, "pitch": None, "roll": None, "updated_at": 0.0}
PRESETS_LOCK = threading.Lock()
THERMAL_LOCK = threading.Lock()
THERMAL_CONDITION = threading.Condition(THERMAL_LOCK)
THERMAL_STATE = {"code": None, "updated_at": 0.0}

THERMAL_PALETTES = {
    "01": {"name": "白热", "sdk": "WHITE_HOT"},
    "03": {"name": "棕褐", "sdk": "SEPIA"},
    "04": {"name": "铁红", "sdk": "IRONBOW"},
    "05": {"name": "彩虹", "sdk": "RAINBOW"},
    "06": {"name": "夜视", "sdk": "NIGHT"},
    "07": {"name": "极光", "sdk": "AURORA"},
    "08": {"name": "红热", "sdk": "RED_HOT"},
    "09": {"name": "丛林", "sdk": "JUNGLE"},
    "0A": {"name": "医疗", "sdk": "MEDICAL"},
    "0B": {"name": "黑热", "sdk": "BLACK_HOT"},
    "0C": {"name": "高亮热", "sdk": "GLORY_HOT"},
}


def with_checksum(command: str) -> bytes:
    checksum = sum(command.encode("ascii")) & 0xFF
    return f"{command}{checksum:02X}".encode("ascii")


def speed_packet(yaw: int, pitch: int) -> bytes:
    command = f"#TPUG4wGSM{yaw & 0xFF:02X}{pitch & 0xFF:02X}"
    return with_checksum(command)


def angle_hex(angle: float) -> str:
    value = max(-9000, min(9000, int(round(float(angle) * 100))))
    return f"{value & 0xFFFF:04X}"


def goto_packet(yaw: float, pitch: float) -> bytes:
    # C12 attitude telemetry reports yaw/pitch with opposite signs from GAM input.
    return with_checksum(f"#TPUGCwGAM{angle_hex(-yaw)}10{angle_hex(-pitch)}10")


def attitude_enable_packet(rate=5) -> bytes:
    return with_checksum(f"#TPUG2wGAA{rate & 0xFF:02X}")


def signed_angle(hex_value: str) -> float:
    value = int(hex_value, 16)
    if value >= 0x8000:
        value -= 0x10000
    return value / 100.0


def parse_attitude(data: bytes):
    try:
        text = data.decode("ascii").strip()
        if not text.startswith("#tpUGCrGAC") or len(text) < 24:
            return None
        body, checksum = text[:-2], text[-2:]
        if (sum(body.encode("ascii")) & 0xFF) != int(checksum, 16):
            return None
        values = body[len("#tpUGCrGAC"):]
        if len(values) < 12:
            return None
        return {
            "yaw": signed_angle(values[0:4]),
            "pitch": signed_angle(values[4:8]),
            "roll": signed_angle(values[8:12]),
            "updated_at": time.time(),
        }
    except (UnicodeDecodeError, ValueError):
        return None


def parse_thermal_palette(data: bytes):
    try:
        text = data.decode("ascii").strip()
        if len(text) < 14:
            return None
        body, checksum = text[:-2], text[-2:]
        if (sum(body.encode("ascii")) & 0xFF) != int(checksum, 16):
            return None
        marker = body.upper().find("IMG")
        if not body.upper().startswith("#TPDU") or marker < 0:
            return None
        code = body[marker + 3:marker + 5].upper()
        return code if code in THERMAL_PALETTES else None
    except (UnicodeDecodeError, ValueError):
        return None


def palette_info(code=None):
    with THERMAL_LOCK:
        current = THERMAL_STATE["code"] if code is None else code
        updated_at = THERMAL_STATE["updated_at"]
    item = THERMAL_PALETTES.get(current)
    return {
        "code": current,
        "name": item["name"] if item else None,
        "sdk": item["sdk"] if item else None,
        "updated_at": updated_at,
    }


def query_thermal_palette(timeout=1.2):
    packet = with_checksum("#TPUD2rIMG00") + b"\r\n"
    sent_at = time.time()
    with CONTROL_LOCK:
        CONTROL_SOCKET.sendto(packet, (CAMERA_IP, CONTROL_PORT))
    with THERMAL_CONDITION:
        THERMAL_CONDITION.wait_for(
            lambda: THERMAL_STATE["updated_at"] >= sent_at, timeout=timeout
        )
    return palette_info()


def set_thermal_palette(code: str):
    code = str(code).upper()
    if code not in THERMAL_PALETTES:
        raise ValueError("不支持的热成像调色板")
    packet = with_checksum(f"#TPUD2wIMG{code}") + b"\r\n"
    with CONTROL_LOCK:
        CONTROL_SOCKET.sendto(packet, (CAMERA_IP, CONTROL_PORT))
    time.sleep(0.12)
    confirmed = query_thermal_palette()
    if confirmed["code"] != code:
        raise OSError("相机未确认调色板切换，请检查 C12 网络连接")
    return confirmed, packet.decode("ascii").strip()


def attitude_receiver():
    last_enable = 0.0
    while not ATTITUDE_STOP.is_set():
        if time.time() - last_enable > 5.0:
            with CONTROL_LOCK:
                CONTROL_SOCKET.sendto(attitude_enable_packet(5), (CAMERA_IP, CONTROL_PORT))
            last_enable = time.time()
        try:
            data, address = CONTROL_SOCKET.recvfrom(4096)
            if address[0] != CAMERA_IP:
                continue
            parsed = parse_attitude(data)
            if parsed:
                with ATTITUDE_LOCK:
                    ATTITUDE.update(parsed)
            palette = parse_thermal_palette(data)
            if palette:
                with THERMAL_CONDITION:
                    THERMAL_STATE.update(code=palette, updated_at=time.time())
                    THERMAL_CONDITION.notify_all()
        except socket.timeout:
            pass
        except OSError:
            break


def current_attitude():
    with ATTITUDE_LOCK:
        result = dict(ATTITUDE)
    result["online"] = result["yaw"] is not None and time.time() - result["updated_at"] < 2.0
    return result


def load_presets():
    try:
        with open(PRESETS_FILE, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def save_presets(presets):
    temporary = PRESETS_FILE + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(presets, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    os.replace(temporary, PRESETS_FILE)


def send_gimbal(action: str, hold=False, speed=None):
    sent = []
    # The official C12 client uses UDP 5000 for both local and remote ports.
    with CONTROL_LOCK:
        if action in DIRECTIONS:
            requested_speed = DEFAULT_HOLD_SPEED if speed is None else int(speed)
            requested_speed = max(8, min(80, requested_speed))
            if not hold:
                requested_speed = STEP_SPEED
            yaw_scale, pitch_scale = DIRECTIONS[action]
            yaw = int(round(yaw_scale * requested_speed))
            pitch = int(round(pitch_scale * requested_speed))
            move = speed_packet(yaw, pitch)
            stop = speed_packet(0, 0)
            CONTROL_SOCKET.sendto(move, (CAMERA_IP, CONTROL_PORT))
            sent.append(move.decode("ascii"))
            if not hold:
                time.sleep(STEP_DURATION)
                CONTROL_SOCKET.sendto(stop, (CAMERA_IP, CONTROL_PORT))
                sent.append(stop.decode("ascii"))
        elif action == "center":
            center = with_checksum(PTZ_CENTER)
            CONTROL_SOCKET.sendto(center, (CAMERA_IP, CONTROL_PORT))
            sent.append(center.decode("ascii"))
        else:
            stop = speed_packet(0, 0)
            CONTROL_SOCKET.sendto(stop, (CAMERA_IP, CONTROL_PORT))
            sent.append(stop.decode("ascii"))
    return sent


class CameraStream:
    def __init__(self, name: str, url: str):
        self.name = name
        self.url = url
        self.frame = None
        self.frame_id = 0
        self.last_frame_at = 0.0
        self.condition = threading.Condition()
        self.process = None
        self.process_started_at = 0.0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)

    def start(self):
        self.thread.start()
        self.watchdog_thread.start()

    def stop(self):
        self.stop_event.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()

    def _watchdog(self):
        # An RTSP TCP socket can remain ESTABLISHED after the camera is
        # unplugged/rebooted while FFmpeg produces no frames. Terminating only
        # that stale child lets _run reconnect without restarting the web UI.
        while not self.stop_event.wait(1.0):
            process = self.process
            reference = max(self.process_started_at, self.last_frame_at)
            if process and process.poll() is None and reference and time.time() - reference > 8.0:
                try:
                    process.terminate()
                except OSError:
                    pass

    def _run(self):
        while not self.stop_event.is_set():
            cmd = [
                "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
                "-rtsp_transport", "tcp", "-i", self.url, "-an",
                "-vf", "fps=12", "-q:v", "5", "-f", "image2pipe",
                "-vcodec", "mjpeg", "pipe:1",
            ]
            try:
                self.process = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    bufsize=0,
                )
                self.process_started_at = time.time()
                buffer = bytearray()
                while not self.stop_event.is_set():
                    chunk = self.process.stdout.read(16384)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    while True:
                        start = buffer.find(b"\xff\xd8")
                        if start < 0:
                            if len(buffer) > 2_000_000:
                                del buffer[:-2]
                            break
                        end = buffer.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            if start:
                                del buffer[:start]
                            break
                        frame = bytes(buffer[start:end + 2])
                        del buffer[:end + 2]
                        with self.condition:
                            self.frame = frame
                            self.frame_id += 1
                            self.last_frame_at = time.time()
                            self.condition.notify_all()
            except (OSError, AttributeError):
                pass
            finally:
                if self.process and self.process.poll() is None:
                    self.process.terminate()
                self.process = None
                self.process_started_at = 0.0
            if not self.stop_event.wait(2.0):
                continue

    def wait_frame(self, previous_id: int, timeout=2.0):
        with self.condition:
            if self.frame_id == previous_id:
                self.condition.wait(timeout)
            return self.frame_id, self.frame

    def online(self):
        return bool(self.frame and time.time() - self.last_frame_at < 3.0)


HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>C12 双光控制台</title>
<style>
:root{color-scheme:dark;--bg:#070b12;--card:#111824;--line:#253247;--text:#e8f0fb;--muted:#91a4bd;--blue:#3ba3ff;--orange:#ff9345;--ok:#35d07f}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -20%,#16283f 0,var(--bg) 48%);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans CJK SC",sans-serif;color:var(--text);min-height:100vh}
header{height:68px;padding:0 24px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #1b2636;background:#080d15d9;backdrop-filter:blur(16px);position:sticky;top:0;z-index:2}
.brand{display:flex;gap:12px;align-items:center}.mark{width:34px;height:34px;border:1px solid #4f698a;border-radius:10px;display:grid;place-items:center;color:var(--blue);font-weight:800}.brand strong{font-size:17px}.sub{font-size:12px;color:var(--muted);margin-top:2px}
.status{font-size:13px;color:var(--muted);display:flex;gap:16px;align-items:center}.dot{width:8px;height:8px;border-radius:50%;background:#637083;display:inline-block;margin-right:6px}.dot.ok{background:var(--ok);box-shadow:0 0 12px #35d07f80}
main{max-width:1500px;margin:0 auto;padding:20px;display:grid;grid-template-columns:minmax(0,1fr) 292px;gap:18px}.views{display:grid;grid-template-columns:1fr 1fr;gap:16px}.card{background:linear-gradient(145deg,#111a28,#0c121c);border:1px solid var(--line);border-radius:16px;overflow:hidden;box-shadow:0 14px 35px #0005}.cardhead{height:46px;display:flex;align-items:center;justify-content:space-between;padding:0 15px;border-bottom:1px solid var(--line)}.title{font-weight:650;font-size:14px}.tag{font-size:11px;color:var(--muted);border:1px solid #304159;border-radius:100px;padding:3px 8px}.view{background:#06090e;aspect-ratio:16/9;display:flex;align-items:center;justify-content:center;position:relative}.view.thermal{aspect-ratio:4/3}.view img{display:block;width:100%;height:100%;object-fit:contain}.label{position:absolute;left:11px;bottom:10px;background:#05080cbf;border:1px solid #ffffff20;padding:5px 8px;border-radius:7px;font:11px ui-monospace,monospace;color:#cfdbeb}
.control{padding:18px;align-self:start;position:sticky;top:88px;max-height:calc(100vh - 108px);overflow-y:auto}.control h2{font-size:15px;margin:0 0 5px}.hint{font-size:12px;color:var(--muted);margin-bottom:18px;line-height:1.5}.dpad{display:grid;grid-template-columns:64px 64px 64px;grid-template-rows:58px 58px 58px;gap:8px;justify-content:center;user-select:none}.btn{border:1px solid #334761;background:#182335;color:#dce9fa;border-radius:12px;font-size:22px;cursor:pointer;transition:.12s;touch-action:none}.btn:hover{border-color:#4c6c93;background:#20314a}.btn:active,.btn.active{transform:scale(.96);background:#1d67a2;border-color:#55b2ff}.up_left{grid-column:1;grid-row:1}.up{grid-column:2;grid-row:1}.up_right{grid-column:3;grid-row:1}.left{grid-column:1;grid-row:2}.center{grid-column:2;grid-row:2;color:var(--orange);font-size:13px;font-weight:700}.right{grid-column:3;grid-row:2}.down_left{grid-column:1;grid-row:3}.down{grid-column:2;grid-row:3}.down_right{grid-column:3;grid-row:3}.speedrow{display:flex;align-items:center;gap:9px;margin-top:15px;color:var(--muted);font-size:12px}.speedrow input{flex:1;accent-color:var(--blue)}.speedvalue{font:11px ui-monospace,monospace;color:#c5d8ed;width:24px;text-align:right}.stop{width:100%;margin-top:12px;height:44px;border:1px solid #71414a;background:#2a171d;color:#ff9ba8;border-radius:11px;font-weight:700;cursor:pointer}.msg{margin-top:14px;min-height:42px;border-top:1px solid var(--line);padding-top:13px;color:var(--muted);font-size:12px;line-height:1.5}.foot{color:#71839a;font-size:11px;margin-top:16px;line-height:1.5}.presets{border-top:1px solid var(--line);margin-top:17px;padding-top:16px}.presets h2{display:flex;justify-content:space-between;align-items:center}.pose{font:11px ui-monospace,monospace;color:var(--muted);font-weight:400}.presetform{display:flex;gap:7px;margin:11px 0}.presetform input{min-width:0;flex:1;background:#0a111b;color:var(--text);border:1px solid #33445b;border-radius:9px;padding:9px 10px;outline:none}.presetform input:focus{border-color:var(--blue)}.savepreset,.callpreset,.smallbtn{border:1px solid #365d82;background:#163451;color:#d9eeff;border-radius:9px;cursor:pointer}.savepreset{padding:0 12px;white-space:nowrap}.presetlist{display:grid;gap:8px}.presetrow{border:1px solid #29384c;background:#0b121d;border-radius:10px;padding:9px}.presetmain{display:flex;align-items:center;justify-content:space-between;gap:6px}.presetname{font-size:13px;font-weight:650;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.presetangle{font:10px ui-monospace,monospace;color:var(--muted);margin-top:4px}.presetactions{display:flex;gap:5px}.callpreset{padding:6px 9px;color:#bfe1ff}.smallbtn{padding:6px 7px;background:#17202d;border-color:#334052;color:#9fb0c5}.empty{font-size:12px;color:#6f829a;text-align:center;padding:12px 0}
.panel{border-top:1px solid var(--line);margin-top:17px;padding-top:16px}.palette-row{display:flex;gap:8px;margin-top:10px}.palette-row select{min-width:0;flex:1;background:#0a111b;color:var(--text);border:1px solid #33445b;border-radius:9px;padding:9px 10px;outline:none}.palette-row select:focus{border-color:var(--blue)}.applypalette{border:1px solid #365d82;background:#163451;color:#d9eeff;border-radius:9px;padding:0 13px;cursor:pointer}.applypalette:disabled{opacity:.55;cursor:wait}.palette-state{font:11px ui-monospace,monospace;color:var(--muted);margin-top:8px}
@media(max-width:1050px){main{grid-template-columns:1fr}.control{position:static}.views{grid-template-columns:1fr 1fr}}@media(max-width:720px){header{padding:0 14px}.status{display:none}main{padding:12px}.views{grid-template-columns:1fr}.control{order:-1}}
</style></head>
<body><header><div class="brand"><div class="mark">IR</div><div><strong>C12 双光控制台</strong><div class="sub">192.168.144.108 · 本机实时预览</div></div></div><div class="status"><span><i id="camdot" class="dot"></i><span id="camstate">检查视频…</span></span><span>控制 UDP 5000</span></div></header>
<main><section class="views">
<article class="card"><div class="cardhead"><span class="title">可见光</span><span class="tag">1280 × 720 · HEVC</span></div><div class="view"><img src="/stream/visible" alt="可见光实时画面"><span class="label">VISIBLE / RTSP 554</span></div></article>
<article class="card"><div class="cardhead"><span class="title">热成像</span><span class="tag">384 × 288 · HEVC</span></div><div class="view thermal"><img src="/stream/thermal" alt="热成像实时画面"><span class="label">THERMAL / RTSP 555</span></div></article>
</section><aside class="card control"><h2>云台八方向控制</h2><div class="hint">按住方向键持续匀速转动，松开立即停止；支持四个斜向和键盘方向键组合。</div>
<div class="dpad"><button class="btn up_left" data-action="up_left" aria-label="左上">↖</button><button class="btn up" data-action="up" aria-label="向上">↑</button><button class="btn up_right" data-action="up_right" aria-label="右上">↗</button><button class="btn left" data-action="left" aria-label="向左">←</button><button class="btn center" data-once="center">回中</button><button class="btn right" data-action="right" aria-label="向右">→</button><button class="btn down_left" data-action="down_left" aria-label="左下">↙</button><button class="btn down" data-action="down" aria-label="向下">↓</button><button class="btn down_right" data-action="down_right" aria-label="右下">↘</button></div>
<div class="speedrow"><span>速度</span><input id="speed" type="range" min="12" max="70" value="36" step="2"><span class="speedvalue" id="speedvalue">36</span></div><button class="stop" id="stop">立即停止</button><div class="msg" id="msg">等待操作</div><div class="foot">控制指令每 250 ms 保活；松开、移出按钮或窗口失焦都会停止。</div>
<section class="panel"><h2>热成像显示样式</h2><div class="hint">选择后点击应用，相机会回读确认实际生效的调色板。</div><div class="palette-row"><select id="palette"><option value="01">白热</option><option value="03">棕褐</option><option value="04">铁红</option><option value="05">彩虹</option><option value="06">夜视</option><option value="07">极光</option><option value="08">红热</option><option value="09">丛林</option><option value="0A">医疗</option><option value="0B">黑热</option><option value="0C">高亮热</option></select><button class="applypalette" id="applypalette">应用</button></div><div class="palette-state" id="palettestate">读取相机当前样式…</div></section>
<section class="presets"><h2><span>预置点</span><span class="pose" id="pose">读取角度…</span></h2><div class="hint">调整到目标视角，输入名称后保存；调用时云台会自动回到该角度。</div><div class="presetform"><input id="presetname" maxlength="40" placeholder="例如：前方、充电桩"><button class="savepreset" id="savepreset">保存</button></div><div class="presetlist" id="presetlist"><div class="empty">暂无预置点</div></div></section></aside></main>
<script>
const msg=document.getElementById('msg'),speed=document.getElementById('speed'),speedvalue=document.getElementById('speedvalue');let activeAction=null,activeButton=null,heartbeat=null;speed.value=localStorage.getItem('c12Speed')||'36';speedvalue.textContent=speed.value;
async function send(action,hold=false,silent=false){try{const r=await fetch('/api/gimbal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,hold,speed:Number(speed.value)})});const j=await r.json();if(!r.ok)throw Error(j.error||'发送失败');if(!silent)msg.textContent=`${new Date().toLocaleTimeString()}  ${j.label}${hold?' · 持续运动':' · 指令已发送'}`;return true}catch(e){msg.textContent='控制失败：'+e.message;return false}}
function clearActive(){if(heartbeat){clearInterval(heartbeat);heartbeat=null}if(activeButton)activeButton.classList.remove('active');activeButton=null;activeAction=null}
function startHold(action,button){if(activeAction===action)return;clearActive();activeAction=action;activeButton=button||document.querySelector(`[data-action="${action}"]`);if(activeButton)activeButton.classList.add('active');send(action,true);heartbeat=setInterval(()=>{if(activeAction)send(activeAction,true,true)},250)}
function stopHold(show=true){const wasActive=!!activeAction;clearActive();if(wasActive||show)send('stop',true,!show)}
document.querySelectorAll('[data-action]').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);startHold(b.dataset.action,b)});b.addEventListener('pointerup',()=>stopHold());b.addEventListener('pointerleave',()=>{if(activeButton===b)stopHold()});b.addEventListener('pointercancel',()=>stopHold());b.addEventListener('lostpointercapture',()=>{if(activeButton===b)stopHold()})});
document.querySelector('[data-once]').addEventListener('click',e=>{stopHold(false);send(e.currentTarget.dataset.once)});document.getElementById('stop').onclick=()=>stopHold();speed.oninput=()=>{speedvalue.textContent=speed.value;localStorage.setItem('c12Speed',speed.value);if(activeAction)send(activeAction,true,true)};
const pressed=new Set(),keymap={ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right'};function keyboardDirection(){const v=pressed.has('up')&&!pressed.has('down')?'up':pressed.has('down')&&!pressed.has('up')?'down':'';const h=pressed.has('left')&&!pressed.has('right')?'left':pressed.has('right')&&!pressed.has('left')?'right':'';return v&&h?`${v}_${h}`:v||h}function updateKeyboard(){const d=keyboardDirection();if(d)startHold(d);else stopHold(false)}document.addEventListener('keydown',e=>{if(keymap[e.key]){e.preventDefault();pressed.add(keymap[e.key]);updateKeyboard()}});document.addEventListener('keyup',e=>{if(keymap[e.key]){e.preventDefault();pressed.delete(keymap[e.key]);updateKeyboard()}});window.addEventListener('blur',()=>{pressed.clear();stopHold(false)});
async function presetRequest(path,data){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const j=await r.json();if(!r.ok)throw Error(j.error||'操作失败');return j}
async function loadPresets(){try{const j=await(await fetch('/api/presets')).json();const a=j.attitude;document.getElementById('pose').textContent=a.online?`Y ${a.yaw.toFixed(1)}° · P ${a.pitch.toFixed(1)}°`:'角度离线';const list=document.getElementById('presetlist');list.replaceChildren();if(!j.presets.length){const e=document.createElement('div');e.className='empty';e.textContent='暂无预置点';list.append(e);return}j.presets.forEach(p=>{const row=document.createElement('div');row.className='presetrow';const main=document.createElement('div');main.className='presetmain';const info=document.createElement('div');info.style.minWidth='0';const name=document.createElement('div');name.className='presetname';name.textContent=p.name;const angle=document.createElement('div');angle.className='presetangle';angle.textContent=`Y ${p.yaw.toFixed(1)}° · P ${p.pitch.toFixed(1)}°`;info.append(name,angle);const actions=document.createElement('div');actions.className='presetactions';const call=document.createElement('button');call.className='callpreset';call.textContent='调用';call.onclick=async()=>{try{await presetRequest('/api/presets/call',{id:p.id});msg.textContent=`正在调用预置点：${p.name}`}catch(e){msg.textContent=e.message}};const rename=document.createElement('button');rename.className='smallbtn';rename.textContent='改名';rename.onclick=async()=>{const n=prompt('新的预置点名称',p.name);if(!n)return;try{await presetRequest('/api/presets/rename',{id:p.id,name:n});await loadPresets()}catch(e){msg.textContent=e.message}};const del=document.createElement('button');del.className='smallbtn';del.textContent='删除';del.onclick=async()=>{if(!confirm(`删除预置点“${p.name}”？`))return;try{await presetRequest('/api/presets/delete',{id:p.id});await loadPresets()}catch(e){msg.textContent=e.message}};actions.append(call,rename,del);main.append(info,actions);row.append(main);list.append(row)})}catch(e){document.getElementById('pose').textContent='角度读取失败'}}
document.getElementById('savepreset').onclick=async()=>{const input=document.getElementById('presetname');try{const j=await presetRequest('/api/presets/save',{name:input.value});input.value='';msg.textContent=`已保存预置点：${j.preset.name}`;await loadPresets()}catch(e){msg.textContent=e.message}};document.getElementById('presetname').addEventListener('keydown',e=>{if(e.key==='Enter')document.getElementById('savepreset').click()});
async function loadPalette(){const state=document.getElementById('palettestate');try{const r=await fetch('/api/thermal/palette');const j=await r.json();if(!r.ok)throw Error(j.error||'读取失败');if(j.palette.code){document.getElementById('palette').value=j.palette.code;state.textContent=`当前：${j.palette.name} · IMG${j.palette.code}`}else state.textContent='暂未读取到相机样式'}catch(e){state.textContent='读取失败：'+e.message}}
document.getElementById('applypalette').onclick=async()=>{const select=document.getElementById('palette'),button=document.getElementById('applypalette'),state=document.getElementById('palettestate');button.disabled=true;state.textContent='正在切换并向相机回读确认…';try{const j=await presetRequest('/api/thermal/palette',{code:select.value});state.textContent=`当前：${j.palette.name} · IMG${j.palette.code}`;msg.textContent=`热成像样式已切换为：${j.palette.name}`}catch(e){state.textContent='切换失败：'+e.message;msg.textContent=state.textContent}finally{button.disabled=false}};
async function health(){try{const j=await(await fetch('/api/status')).json();const ok=j.visible&&j.thermal;document.getElementById('camdot').className='dot '+(ok?'ok':'');document.getElementById('camstate').textContent=ok?'双路视频在线':`可见光 ${j.visible?'在线':'等待'} / 热成像 ${j.thermal?'在线':'等待'}`}catch(e){document.getElementById('camstate').textContent='服务离线'}}health();setInterval(health,3000);
loadPresets();setInterval(loadPresets,3000);
loadPalette();
</script></body></html>'''

LABELS = {
    "up": "向上", "down": "向下", "left": "向左", "right": "向右",
    "up_left": "左上", "up_right": "右上", "down_left": "左下", "down_right": "右下",
    "stop": "停止", "center": "回中",
}
CAMERAS = {name: CameraStream(name, url) for name, url in STREAMS.items()}


class Handler(BaseHTTPRequestHandler):
    server_version = "C12Control/1.0"

    def log_message(self, fmt, *args):
        print(f"[{self.log_date_time_string()}] {fmt % args}", flush=True)

    def _json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            data = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        elif path == "/api/status":
            status = {name: cam.online() for name, cam in CAMERAS.items()}
            status["camera_ip"] = CAMERA_IP
            status["attitude"] = current_attitude()
            self._json(200, status)
        elif path == "/api/presets":
            with PRESETS_LOCK:
                presets = load_presets()
            self._json(200, {"presets": presets, "attitude": current_attitude()})
        elif path == "/api/thermal/palette":
            palette = query_thermal_palette()
            self._json(200, {
                "palette": palette,
                "palettes": [{"code": code, **item} for code, item in THERMAL_PALETTES.items()],
            })
        elif path.startswith("/snapshot/"):
            name = path.rsplit("/", 1)[-1]
            cam = CAMERAS.get(name)
            frame = cam.frame if cam else None
            if not frame:
                self.send_error(503, "camera frame unavailable")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(frame)
        elif path.startswith("/stream/"):
            name = path.rsplit("/", 1)[-1]
            cam = CAMERAS.get(name)
            if cam is None:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            frame_id = -1
            try:
                while True:
                    frame_id, frame = cam.wait_frame(frame_id)
                    if not frame:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                    self.wfile.write(frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            data = self._read_json()
            if path == "/api/gimbal":
                action = data.get("action")
                if action not in ACTIONS:
                    self._json(400, {"error": "不支持的控制动作"})
                    return
                hold = bool(data.get("hold", False))
                speed = data.get("speed", DEFAULT_HOLD_SPEED)
                packets = send_gimbal(action, hold=hold, speed=speed)
                self._json(200, {"ok": True, "action": action, "label": LABELS[action], "packets": packets})
            elif path == "/api/thermal/palette":
                palette, packet = set_thermal_palette(data.get("code", ""))
                self._json(200, {"ok": True, "palette": palette, "packet": packet})
            elif path == "/api/presets/save":
                name = str(data.get("name", "")).strip()[:40]
                attitude = current_attitude()
                if not name:
                    self._json(400, {"error": "请输入预置点名称"})
                    return
                if not attitude["online"]:
                    self._json(503, {"error": "暂未收到云台角度，请稍后再试"})
                    return
                preset = {
                    "id": uuid.uuid4().hex,
                    "name": name,
                    "yaw": round(attitude["yaw"], 2),
                    "pitch": round(attitude["pitch"], 2),
                    "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                }
                with PRESETS_LOCK:
                    presets = load_presets()
                    presets.append(preset)
                    save_presets(presets)
                self._json(200, {"ok": True, "preset": preset})
            elif path == "/api/presets/call":
                preset_id = str(data.get("id", ""))
                with PRESETS_LOCK:
                    presets = load_presets()
                preset = next((item for item in presets if item.get("id") == preset_id), None)
                if preset is None:
                    self._json(404, {"error": "预置点不存在"})
                    return
                packet = goto_packet(preset["yaw"], preset["pitch"])
                with CONTROL_LOCK:
                    CONTROL_SOCKET.sendto(packet, (CAMERA_IP, CONTROL_PORT))
                self._json(200, {"ok": True, "preset": preset, "packet": packet.decode("ascii")})
            elif path == "/api/presets/delete":
                preset_id = str(data.get("id", ""))
                with PRESETS_LOCK:
                    presets = load_presets()
                    updated = [item for item in presets if item.get("id") != preset_id]
                    if len(updated) == len(presets):
                        self._json(404, {"error": "预置点不存在"})
                        return
                    save_presets(updated)
                self._json(200, {"ok": True})
            elif path == "/api/presets/rename":
                preset_id = str(data.get("id", ""))
                name = str(data.get("name", "")).strip()[:40]
                if not name:
                    self._json(400, {"error": "名称不能为空"})
                    return
                found = False
                with PRESETS_LOCK:
                    presets = load_presets()
                    for preset in presets:
                        if preset.get("id") == preset_id:
                            preset["name"] = name
                            found = True
                            break
                    if found:
                        save_presets(presets)
                if not found:
                    self._json(404, {"error": "预置点不存在"})
                    return
                self._json(200, {"ok": True})
            else:
                self.send_error(404)
        except (ValueError, OSError) as exc:
            self._json(500, {"error": str(exc)})


def shutdown(*_):
    for camera in CAMERAS.values():
        camera.stop()
    ATTITUDE_STOP.set()
    try:
        with CONTROL_LOCK:
            CONTROL_SOCKET.sendto(attitude_enable_packet(0), (CAMERA_IP, CONTROL_PORT))
    except OSError:
        pass
    CONTROL_SOCKET.close()
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    for camera in CAMERAS.values():
        camera.start()
    threading.Thread(target=attitude_receiver, daemon=True).start()
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), Handler)
    server.daemon_threads = True
    print(f"C12 console: http://{HTTP_HOST}:{HTTP_PORT} (camera {CAMERA_IP})", flush=True)
    try:
        server.serve_forever()
    finally:
        shutdown()
