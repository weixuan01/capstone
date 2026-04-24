import asyncio
import os
import pty
import fcntl
import termios
import struct
import json
import signal
import threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path

import rclpy
from rclpy.node import Node as RclNode
from std_msgs.msg import String

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

HTML_FILE = Path(__file__).parent / "index.html"
active_launch_pid = None

ROS2_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP   = os.path.expanduser("~/capstone_beta/core/crazyflie_mapping_demo/ros2_ws/install/setup.bash")

# ── Shared ROS bridge ──────────────────────────────────────────────────────
#
# A single long-lived rclpy node subscribes to /battery_status and publishes
# to /land_command and /recall_command. This replaces the previous pattern
# of spawning `ros2 topic echo --once` / `ros2 topic pub --once` subprocesses,
# which created hundreds of transient _ros2cli_XXXXX nodes in the ROS graph.

_latest_battery: dict = {}
_battery_lock = threading.Lock()
_ros_bridge = None
_ros_ready = threading.Event()


class RosBridgeNode(RclNode):
    def __init__(self):
        super().__init__('mission_control_web_bridge')
        self.create_subscription(String, '/battery_status', self._on_battery, 10)
        self.land_pub   = self.create_publisher(String, '/land_command', 10)
        self.recall_pub = self.create_publisher(String, '/recall_command', 10)

    def _on_battery(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                with _battery_lock:
                    _latest_battery.clear()
                    _latest_battery.update(data)
        except Exception:
            pass

    def publish_land(self, prefixes):
        msg = String()
        msg.data = json.dumps({"prefixes": list(prefixes)})
        self.land_pub.publish(msg)

    def publish_recall(self, prefixes):
        msg = String()
        msg.data = json.dumps({"prefixes": list(prefixes)})
        self.recall_pub.publish(msg)


def _spin_ros_bridge():
    global _ros_bridge
    try:
        rclpy.init(args=None)
    except RuntimeError:
        # Already initialised elsewhere — fine.
        pass
    _ros_bridge = RosBridgeNode()
    _ros_ready.set()
    try:
        rclpy.spin(_ros_bridge)
    finally:
        try:
            _ros_bridge.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


@app.on_event("startup")
async def _startup():
    threading.Thread(target=_spin_ros_bridge, daemon=True).start()
    # Give the bridge a moment to come up. Not strictly required — the
    # battery WebSocket will simply see an empty snapshot until it does.
    _ros_ready.wait(timeout=5.0)


@app.get("/")
async def get():
    return HTMLResponse(HTML_FILE.read_text())

# ── Terminal WebSocket ──────────────────────────────────────────────────────

@app.websocket("/ws/terminal")
async def terminal_ws(websocket: WebSocket):
    await websocket.accept()
    master_fd, slave_fd = pty.openpty()
    pid = os.fork()
    if pid == 0:
        os.setsid()
        os.dup2(slave_fd, 0); os.dup2(slave_fd, 1); os.dup2(slave_fd, 2)
        os.close(master_fd); os.close(slave_fd)
        os.execvp("bash", ["bash"])
        os._exit(1)
    os.close(slave_fd)
    fcntl.fcntl(master_fd, fcntl.F_SETFL, fcntl.fcntl(master_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
    loop = asyncio.get_event_loop()

    async def read_output():
        while True:
            try:
                data = await loop.run_in_executor(None, lambda: _read_fd(master_fd))
                if data:
                    await websocket.send_text(json.dumps({"type": "output", "data": data.decode("utf-8", errors="replace")}))
                else:
                    await asyncio.sleep(0.01)
            except Exception:
                break

    output_task = asyncio.create_task(read_output())
    try:
        while True:
            raw = await websocket.receive_text()
            msg = json.loads(raw)
            if msg["type"] == "input":
                os.write(master_fd, msg["data"].encode("utf-8"))
            elif msg["type"] == "resize":
                fcntl.ioctl(master_fd, termios.TIOCSWINSZ,
                    struct.pack("HHHH", msg.get("rows", 24), msg.get("cols", 80), 0, 0))
    except WebSocketDisconnect:
        pass
    finally:
        output_task.cancel()
        try: os.kill(pid, 9); os.waitpid(pid, 0)
        except Exception: pass
        os.close(master_fd)

# ── Launch WebSocket ────────────────────────────────────────────────────────

@app.websocket("/ws/launch")
async def launch_ws(websocket: WebSocket):
    global active_launch_pid
    await websocket.accept()
    loop = asyncio.get_event_loop()

    async def stream_process(cmd: str):
        global active_launch_pid
        master_fd, slave_fd = pty.openpty()
        pid = os.fork()
        if pid == 0:
            os.setsid()
            os.dup2(slave_fd, 0); os.dup2(slave_fd, 1); os.dup2(slave_fd, 2)
            os.close(master_fd); os.close(slave_fd)
            os.execvp("bash", ["bash", "-c", cmd])
            os._exit(1)
        os.close(slave_fd)
        active_launch_pid = pid
        fcntl.fcntl(master_fd, fcntl.F_SETFL, fcntl.fcntl(master_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        await websocket.send_text(json.dumps({"type": "status", "value": "running"}))
        while True:
            data = await loop.run_in_executor(None, lambda: _read_fd(master_fd))
            if data:
                await websocket.send_text(json.dumps({"type": "output", "data": data.decode("utf-8", errors="replace")}))
            else:
                r = os.waitpid(pid, os.WNOHANG)
                if r[0] != 0:
                    break
                await asyncio.sleep(0.05)
        os.close(master_fd)
        active_launch_pid = None
        await websocket.send_text(json.dumps({"type": "status", "value": "stopped"}))

    try:
        while True:
            raw = await websocket.receive_text()
            msg = json.loads(raw)
            if msg["type"] == "launch":
                cmd = build_launch_command(msg["config"])
                await websocket.send_text(json.dumps({"type": "cmd", "data": cmd}))
                asyncio.create_task(stream_process(cmd))
            elif msg["type"] == "stop":
                if active_launch_pid:
                    try: os.killpg(os.getpgid(active_launch_pid), signal.SIGINT)
                    except Exception: pass
    except WebSocketDisconnect:
        pass

# ── Battery WebSocket ────────────────────────────────────────────────────────
#
# Serves the latest /battery_status snapshot from the shared rclpy bridge.
# No subprocess, no ros2 topic echo, no transient CLI nodes in the graph.

@app.websocket("/ws/battery")
async def battery_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with _battery_lock:
                snapshot = dict(_latest_battery)
            if snapshot:
                await websocket.send_text(json.dumps(snapshot))
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass

# ── Land WebSocket ───────────────────────────────────────────────────────────
#
# Accepts: { "type": "land",       "prefix": "cf1" }
#          { "type": "land_all",   "prefixes": ["cf1", "cf2"] }
#          { "type": "recall",     "prefix": "cf1" }
#          { "type": "recall_all", "prefixes": ["cf1", "cf2"] }
#
# land / land_all: publishes to /land_command — mission_control forwards a
#   NaN Point with z=1.0 to /cfX/assigned_goal so the drone lands in place.
#
# recall / recall_all: publishes to /recall_command — mission_control forwards
#   a NaN Point with z=0.0 to /cfX/assigned_goal so the drone returns home
#   then lands.
#
# All use the persistent rclpy bridge publisher — no subprocesses.

@app.websocket("/ws/land")
async def land_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            raw = await websocket.receive_text()
            msg = json.loads(raw)

            if msg["type"] in ("recall", "recall_all"):
                if msg["type"] == "recall":
                    prefixes = [msg.get("prefix", "").lstrip("/")]
                else:
                    prefixes = [p.lstrip("/") for p in msg.get("prefixes", [])]

                prefixes = [p for p in prefixes if p]
                if prefixes and _ros_bridge is not None:
                    try:
                        _ros_bridge.publish_recall(prefixes)
                    except Exception as e:
                        print(f"[recall] publish failed: {e}", flush=True)
                    await websocket.send_text(json.dumps({"type": "ack_recall", "prefixes": prefixes}))

            else:
                # Land in place (land or land_all)
                prefixes = []
                if msg["type"] == "land":
                    prefixes = [msg.get("prefix", "").lstrip("/")]
                elif msg["type"] == "land_all":
                    prefixes = [p.lstrip("/") for p in msg.get("prefixes", [])]

                prefixes = [p for p in prefixes if p]

                if prefixes and _ros_bridge is not None:
                    try:
                        _ros_bridge.publish_land(prefixes)
                    except Exception as e:
                        print(f"[land] publish failed: {e}", flush=True)
                    await websocket.send_text(json.dumps({"type": "ack", "prefixes": prefixes}))

    except WebSocketDisconnect:
        pass

# ── Command builder ─────────────────────────────────────────────────────────

SCRIPT_UNIVERSAL = os.path.expanduser("~/capstone_beta/scripts/launch-universal.sh")
SCRIPT_BASIC     = os.path.expanduser("~/capstone_beta/scripts/launch.sh")

BASIC_MISSIONS   = {"wallfollowing", "manual", "square"}

def build_launch_command(config: dict) -> str:
    mode = config.get("mode", "sim")
    drones = config.get("drones", [])
    if not drones:
        return ""

    first_mission = drones[0].get("mission", "")
    source_prefix = f"source {ROS2_SETUP} && source {WS_SETUP} && "

    if first_mission in BASIC_MISSIONS:
        extra = drones[0].get("extra", "").strip()
        cmd = f"bash {SCRIPT_BASIC} {mode} {first_mission}"
        if extra:
            cmd += f" {extra}"
        return source_prefix + cmd
    else:
        specs = []
        for d in drones:
            prefix  = d.get("prefix", "cf1").lstrip("/")
            mission = d.get("mission", "frontier-exploration")
            extra   = d.get("extra", "").strip()
            spec    = f"{prefix}:{mission}" + (f":{extra}" if extra else "")
            specs.append(spec)
        drone_str = " ".join(specs)
        return source_prefix + f"bash {SCRIPT_UNIVERSAL} {mode} {drone_str}"

def _read_fd(fd):
    import select
    r, _, _ = select.select([fd], [], [], 0.05)
    if r:
        try: return os.read(fd, 4096)
        except OSError: return None
    return None
