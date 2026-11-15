#!/usr/bin/env python3
"""
server.py — веб-пульт для LLM-танка.

Бэкенд: держит Bluetooth-связь с танком, исполняет LLM-задания и ручные
команды, ведёт путевую карту (dead reckoning) и рассылает события по WebSocket.

Запуск:   python3 server.py [--port 7100]
Mock-режим без железа: TANK_MOCK=1 python3 server.py
"""

import asyncio
import json
import math
import os
import platform
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/..")

import serial  # noqa: E402
import uvicorn  # noqa: E402
from fastapi import FastAPI, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse  # noqa: E402

IS_MAC = platform.system() == "Darwin"

import bridge  # noqa: E402  (llm_tank_bridge/bridge.py)

# ---------------- Настройки ----------------

BT_ADDR = bridge.BT_ADDR
BT_PIN = bridge.BT_PIN
SERIAL_PORT = bridge.SERIAL_PORT
BAUD = bridge.BAUD
MOCK = os.environ.get("TANK_MOCK") == "1"

# Калибровка одометрии (приблизительно, dead reckoning без энкодеров!)
SPEED_CM_S = 30.0          # скорость вперёд, см/с
TURN_DEG_PER_MS = 0.12     # градусов поворота за мс (300 мс ≈ 36°)
SCAN_OFFSETS = (-70, -30, 0, 30, 70)   # углы лучей сканера относительно курса

BASE = os.path.dirname(os.path.abspath(__file__))

# ---------------- Состояние ----------------

pose = {"x": 0.0, "y": 0.0, "heading": 0.0}   # см, градусы; 0 = «север»
obstacles = []                                 # [(x, y), ...] из сканов
trail = []                                     # [(x, y), ...] путь
clients = set()
loop = None
ser = None
tank_lock = threading.Lock()
agent_thread = None
agent_stop = threading.Event()

# ---------------- Рассылка ----------------

def broadcast(event: dict):
    if loop:
        asyncio.run_coroutine_threadsafe(_broadcast_async(event), loop)

async def _broadcast_async(event: dict):
    dead = []
    data = json.dumps(event, ensure_ascii=False)
    for ws in clients:
        try:
            await ws.send_text(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)

def log(direction: str, text: str):
    broadcast({"type": "log", "dir": direction, "text": text,
               "ts": time.strftime("%H:%M:%S")})

def map_event(kind: str):
    broadcast({"type": "map", "kind": kind, "pose": dict(pose),
               "trail": trail[-500:], "obstacles": obstacles[-500:]})

# ---------------- Танк ----------------

class RFCOMMLink:
    """Linux: прямой RFCOMM-сокет к HC-06 без /dev/rfcomm.
    Имитирует нужные методы serial.Serial (write/readline/reset_input_buffer)."""

    def __init__(self, mac: str, channel: int = 1, timeout: float = 12.0):
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                  socket.BTPROTO_RFCOMM)
        self.sock.settimeout(timeout)
        self.sock.connect((mac, channel))
        self.buf = b""

    def reset_input_buffer(self):
        self.buf = b""
        self.sock.setblocking(False)
        try:
            while self.sock.recv(256):
                pass
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.settimeout(12.0)

    def write(self, data: bytes):
        self.sock.sendall(data)

    def readline(self) -> bytes:
        while b"\n" not in self.buf:
            chunk = self.sock.recv(256)
            if not chunk:
                raise ConnectionError("RFCOMM-соединение закрыто танком")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line + b"\n"

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

def bt_connect_linux() -> bool:
    """На Linux устройство уже спарено (bluetoothctl pair/trust) —
    просто проверяем, что BlueZ его знает."""
    r = subprocess.run(["bluetoothctl", "info", BT_ADDR],
                       capture_output=True, text=True)
    if "Paired: yes" not in r.stdout:
        log("sys", "HC-06 не спарен на этом хосте! "
                   "Выполните: bluetoothctl pair " + BT_ADDR)
        return False
    return True

def bt_connect():
    """Переспаривание HC-06 и подключение (обход глюка клонов на macOS)."""
    def bt(*args):
        return subprocess.run(["blueutil", *args], capture_output=True, text=True)
    log("sys", "Bluetooth: переспаривание HC-06...")
    bt("--disconnect", BT_ADDR)
    bt("--unpair", BT_ADDR)
    time.sleep(3)
    bt("--pair", BT_ADDR, BT_PIN)
    time.sleep(2)
    for attempt in range(6):
        bt("--connect", BT_ADDR)
        for _ in range(24):
            if bt("--is-connected", BT_ADDR).stdout.strip() == "1":
                break
            time.sleep(0.5)
        for _ in range(24):
            if os.path.exists(SERIAL_PORT):
                break
            time.sleep(0.5)
        if bt("--is-connected", BT_ADDR).stdout.strip() == "1" and os.path.exists(SERIAL_PORT):
            log("sys", f"Bluetooth соединён (попытка {attempt + 1})")
            return True
        time.sleep(3)
    return False

def tank_open():
    global ser
    if MOCK:
        log("sys", "MOCK-режим: железо не используется")
        return
    if IS_MAC:
        if not bt_connect():
            raise ConnectionError("Bluetooth не поднялся")
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=12)
    else:
        # Linux: RFCOMM-сокет напрямую, устройство спарено заранее
        if not bt_connect_linux():
            raise ConnectionError("HC-06 не спарен с этим хостом")
        last_err = None
        for attempt in range(6):
            try:
                log("sys", f"RFCOMM: подключение к {BT_ADDR} (попытка {attempt + 1})...")
                ser = RFCOMMLink(BT_ADDR, channel=1, timeout=12)
                break
            except OSError as e:
                last_err = e
                time.sleep(3)
        else:
            raise ConnectionError(f"RFCOMM не поднялся: {last_err}")
    time.sleep(2)
    ser.reset_input_buffer()
    greeting = ser.readline().decode(errors="replace").strip()
    log("sys", f"Порт открыт. Приветствие: {greeting or '(нет)'}")
    log("tx", "PING")
    log("rx", tank_raw("PING", quiet=True))

def tank_raw(command: str, quiet: bool = False) -> str:
    """Низкоуровневая отправка. Вызывать под tank_lock."""
    if MOCK:
        return mock_response(command)
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    return ser.readline().decode(errors="replace").strip()

def mock_response(command: str) -> str:
    time.sleep(0.15)
    op = command.split()[0]
    if op == "PING":
        return "PONG"
    if op == "SCAN":
        import random
        return "DIST:" + ",".join(str(random.choice([15, 40, 80, 150, 400])) for _ in range(5))
    if op == "IR":
        return "IR:0,0"
    if op == "STATUS":
        return "SPEED:168/255 CENTER:107 MAXDUR:3000"
    return "OK"

def update_pose(command: str, response: str):
    """Dead reckoning: двигаем робота на карте по выполненной команде."""
    parts = command.split()
    op = parts[0]
    ms = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0

    if op in ("FWD", "BACK") and response == "OK":
        dist = SPEED_CM_S * ms / 1000.0
        if op == "BACK":
            dist = -dist
        pose["x"] += dist * math.sin(math.radians(pose["heading"]))
        pose["y"] += dist * math.cos(math.radians(pose["heading"]))
        trail.append((round(pose["x"], 1), round(pose["y"], 1)))
        map_event("move")
    elif op == "LEFT" and response == "OK":
        pose["heading"] -= TURN_DEG_PER_MS * ms
        map_event("move")
    elif op == "RIGHT" and response == "OK":
        pose["heading"] += TURN_DEG_PER_MS * ms
        map_event("move")
    elif op in ("FWD",) and response == "STOPPED_IR":
        # препятствие перед собой: ставим метку в ~10 см по курсу
        ox = pose["x"] + 10 * math.sin(math.radians(pose["heading"]))
        oy = pose["y"] + 10 * math.cos(math.radians(pose["heading"]))
        obstacles.append((round(ox, 1), round(oy, 1)))
        map_event("obstacle")

def record_scan(resp: str):
    """SCAN-лучи -> точки препятствий на карте."""
    if not resp.startswith("DIST:"):
        return
    try:
        dists = [float(v) for v in resp[5:].split(",")]
    except ValueError:
        return
    for off, d in zip(SCAN_OFFSETS, dists):
        if d >= 400:
            continue
        ang = math.radians(pose["heading"] + off)
        obstacles.append((round(pose["x"] + d * math.sin(ang), 1),
                          round(pose["y"] + d * math.cos(ang), 1)))
    map_event("scan")

def tank_cmd(command: str, source: str = "manual") -> str:
    """Потокобезопасная команда с логом и картой."""
    with tank_lock:
        log("tx", command)
        resp = tank_raw(command)
        log("rx", resp or "(молчит)")
    update_pose(command, resp)
    if command == "SCAN":
        record_scan(resp)
    return resp

# ---------------- LLM-агент ----------------

class WebTank:
    """Адаптер под интерфейс bridge.Tank, но через tank_cmd с картой и логом."""
    def cmd(self, command: str) -> str:
        return tank_cmd(command, source="agent")

def agent_run(task: str):
    broadcast({"type": "agent", "state": "running"})
    log("sys", f"Задание LLM: {task}")
    tank = WebTank()
    messages = [{"role": "system", "content": bridge.SYSTEM_PROMPT},
                {"role": "user", "content": task}]
    try:
        for _ in range(40):
            if agent_stop.is_set():
                tank_cmd("STOP")
                log("sys", "Агент остановлен пользователем")
                break
            msg = bridge.chat(messages)
            messages.append(msg)
            tcs = msg.get("tool_calls") or []
            if not tcs:
                log("llm", msg.get("content", "").strip())
                break
            for tc in tcs:
                if agent_stop.is_set():
                    break
                fn = tc["function"]
                result = bridge.run_tool(tank, fn["name"], fn.get("arguments") or {})
                tool_msg = {"role": "tool", "name": fn["name"], "content": result}
                if tc.get("id"):
                    tool_msg["tool_call_id"] = tc["id"]
                messages.append(tool_msg)
        else:
            tank_cmd("STOP")
            log("sys", "Лимит шагов исчерпан, танк остановлен")
    except Exception as e:
        log("sys", f"Ошибка агента: {e}")
    finally:
        broadcast({"type": "agent", "state": "idle"})

# ---------------- FastAPI ----------------

app = FastAPI()

@app.get("/")
async def index():
    return FileResponse(os.path.join(BASE, "index.html"))

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps(
        {"type": "map", "kind": "full", "pose": pose, "trail": trail,
         "obstacles": obstacles}, ensure_ascii=False))
    try:
        while True:
            data = json.loads(await ws.receive_text())
            kind = data.get("type")
            if kind == "cmd":                       # ручное управление
                cmd = data.get("cmd", "")
                allowed = {"FWD", "BACK", "LEFT", "RIGHT", "STOP", "SCAN", "IR"}
                if cmd.split()[0] in allowed:
                    threading.Thread(target=tank_cmd, args=(cmd,), daemon=True).start()
            elif kind == "task":                    # задание для LLM
                global agent_thread
                if agent_thread and agent_thread.is_alive():
                    log("sys", "Агент уже выполняет задание — сначала «Стоп агент»")
                else:
                    agent_stop.clear()
                    agent_thread = threading.Thread(
                        target=agent_run, args=(data.get("text", ""),), daemon=True)
                    agent_thread.start()
            elif kind == "agent_stop":
                agent_stop.set()
            elif kind == "map_reset":
                pose.update(x=0.0, y=0.0, heading=0.0)
                obstacles.clear()
                trail.clear()
                map_event("full")
    except WebSocketDisconnect:
        clients.discard(ws)

@app.on_event("startup")
async def startup():
    global loop
    loop = asyncio.get_running_loop()
    threading.Thread(target=_connect_thread, daemon=True).start()

def _connect_thread():
    try:
        tank_open()
        broadcast({"type": "link", "state": "ok"})
    except Exception as e:
        log("sys", f"Не удалось подключиться к танку: {e}")
        broadcast({"type": "link", "state": "fail"})

if __name__ == "__main__":
    port = 7100
    if "--port" in sys.argv:
        port = int(sys.argv[sys.argv.index("--port") + 1])
    port = int(os.environ.get("PORT", port))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")
