#!/usr/bin/env python3
"""
server.py — веб-пульт для LLM-танка.

Бэкенд: держит Bluetooth-связь с танком, исполняет LLM-задания и ручные
команды, ведёт путевую карту (dead reckoning) и рассылает события по WebSocket.

Запуск:   python3 server.py [--port 7100]
Mock-режим без железа: TANK_MOCK=1 python3 server.py
"""

import asyncio
from collections import deque
import json
import math
import os
import platform
import re
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
# Поддерживается только адаптер BCM20702A0, для которого настроен sudoers.
USB_RESET_COMMAND = ["sudo", "-n", "/usr/bin/usbreset", "0b05:17cb"]

# Калибровка одометрии (приблизительно, dead reckoning без энкодеров!)
SPEED_CM_S = 30.0          # скорость вперёд, см/с
TURN_DEG_PER_MS = 0.12     # градусов поворота за мс (300 мс ≈ 36°)
SCAN_OFFSETS = (-70, -30, 0, 30, 70)   # углы лучей сканера относительно курса

# Вольтметр АКБ (делитель 1:1 на A0). Включается переменной TANK_BATT=1.
# Пороги для 2×18650 по умолчанию: 8.4 В полный, 6.0 В пустой.
BATT_ENABLED = os.environ.get("TANK_BATT", "0") == "1"
BATT_FULL_MV = int(os.environ.get("TANK_BATT_FULL", "8400"))
BATT_EMPTY_MV = int(os.environ.get("TANK_BATT_EMPTY", "6000"))

BASE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(BASE, "settings.json")
CAL_FILE = os.path.join(BASE, "calibration.json")   # калибровка, переживает перезагрузку танка

def cal_load() -> dict:
    try:
        with open(CAL_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def cal_save(cal: dict):
    try:
        with open(CAL_FILE, "w", encoding="utf-8") as f:
            json.dump(cal, f)
    except Exception as e:
        log("sys", f"Не удалось сохранить калибровку: {e}")

# ---------------- Состояние ----------------

pose = {"x": 0.0, "y": 0.0, "heading": 0.0}   # см, градусы; 0 = «север»
obstacles = []                                 # [(x, y), ...] из сканов
trail = []                                     # [(x, y), ...] путь
clients = set()
log_history = deque(maxlen=300)
log_lock = threading.Lock()
repair_lock = threading.Lock()
loop = None
ser = None
link_state = None          # None / "ok" / "fail" — чтобы не слать дубли
tank_lock = threading.Lock()
agent_thread = None
agent_stop = threading.Event()
DEFAULT_HEARTBEAT_S = 60
HEARTBEAT_MIN_S = 10
HEARTBEAT_MAX_S = 3600
heartbeat_changed = threading.Event()
settings_lock = threading.Lock()

def load_heartbeat_interval():
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            value = json.load(f).get("heartbeat_s", DEFAULT_HEARTBEAT_S)
        if type(value) is int and HEARTBEAT_MIN_S <= value <= HEARTBEAT_MAX_S:
            return value
    except (OSError, ValueError, AttributeError):
        pass
    return DEFAULT_HEARTBEAT_S

heartbeat_interval = load_heartbeat_interval()

def set_heartbeat_interval(value):
    global heartbeat_interval
    if type(value) is not int or not HEARTBEAT_MIN_S <= value <= HEARTBEAT_MAX_S:
        raise ValueError(f"Укажите целое число от {HEARTBEAT_MIN_S} до {HEARTBEAT_MAX_S} секунд")
    with settings_lock:
        temp_path = SETTINGS_FILE + ".tmp"
        try:
            with open(temp_path, "w", encoding="utf-8") as f:
                json.dump({"heartbeat_s": value}, f)
            os.replace(temp_path, SETTINGS_FILE)
        except OSError:
            try:
                os.unlink(temp_path)
            except OSError:
                pass
            raise
        heartbeat_interval = value
        heartbeat_changed.set()
    log("sys", f"Интервал проверки связи: {value} с")
    broadcast({"type": "settings", "heartbeat_s": value})

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
    event = {"type": "log", "dir": direction, "text": text,
             "ts": time.strftime("%H:%M:%S")}
    with log_lock:
        log_history.append(event)
        print(f"[{event['ts']}] {direction}: {text}", flush=True)
        broadcast(event)

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
        try:
            self.sock.connect((mac, channel))
        except Exception:
            self.sock.close()
            raise
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

def bt_info_linux() -> str:
    r = subprocess.run(["bluetoothctl", "info", BT_ADDR],
                       capture_output=True, text=True, timeout=10)
    output = (r.stdout + r.stderr).strip()
    log("sys", f"bluetoothctl info: код {r.returncode}; {output or '(нет вывода)'}")
    return r.stdout


def bt_connect_linux() -> bool:
    return "Paired: yes" in bt_info_linux()


def bt_repair_linux() -> bool:
    """Ручное переспаривание; вывод bluetoothctl сохраняется без PIN."""
    log("sys", "Переспаривание: состояние перед удалением:")
    try:
        bt_info_linux()
        r = subprocess.run(["bluetoothctl", "remove", BT_ADDR],
                           capture_output=True, text=True, timeout=10)
        log("sys", f"bluetoothctl remove: код {r.returncode}; {(r.stdout + r.stderr).strip() or '(нет вывода)'}")
        proc = subprocess.Popen(["bluetoothctl"], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1)
        discovered = threading.Event()
        pin_requested = threading.Event()
        paired = threading.Event()
        def read_output():
            for line in proc.stdout:
                # PIN может быть повторён программой или терминалом.
                line = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", line).strip()
                if line:
                    if BT_ADDR.upper() in line.upper() and ("[NEW]" in line or "[CHG]" in line):
                        discovered.set()
                    if "Request PIN code" in line:
                        pin_requested.set()
                    if "Pairing successful" in line or (BT_ADDR.upper() in line.upper() and "Paired: yes" in line):
                        paired.set()
                    log("sys", "bluetoothctl: " + line.replace(BT_PIN, "[PIN скрыт]"))
        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        try:
            def send(cmd, wait, label=None):
                log("sys", f"bluetoothctl → {label or cmd}")
                proc.stdin.write(cmd + "\n")
                proc.stdin.flush()
                time.sleep(wait)
            send("power on", 1)
            send("agent KeyboardDisplay", 1)
            send("default-agent", 1)
            send("scan bredr", 0)
            log("sys", "Поиск HC-06: ожидаю обнаружения до 60 с")
            if not discovered.wait(60):
                log("sys", "HC-06 не обнаружен за 60 с; сопряжение не запускалось")
                send("scan off", 1)
                send("quit", 0)
                proc.wait(timeout=10)
                reader.join(timeout=2)
                return False
            log("sys", "HC-06 обнаружен; начинаю сопряжение")
            send(f"pair {BT_ADDR}", 0)
            deadline = time.monotonic() + 20
            while not paired.is_set() and not pin_requested.is_set() and time.monotonic() < deadline:
                time.sleep(0.1)
            if pin_requested.is_set() and not paired.is_set():
                send(BT_PIN, 0, "[PIN скрыт]")
            if not paired.wait(20):
                log("sys", "Нет подтверждения сопряжения от BlueZ")
                send("scan off", 1)
                send("quit", 0)
                proc.wait(timeout=10)
                reader.join(timeout=2)
                return False
            send(f"trust {BT_ADDR}", 2)
            send("scan off", 1)
            send("quit", 1)
            proc.wait(timeout=10)
            reader.join(timeout=2)
            log("sys", f"bluetoothctl завершён: код {proc.returncode}")
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        info = bt_info_linux()
        if "Paired: yes" not in info or "Trusted: yes" not in info:
            log("sys", "Переспаривание не подтверждено BlueZ; проверьте вывод выше")
            return False
        return True
    except Exception as e:
        log("sys", f"Переспаривание: {type(e).__name__}: {e}")
        return False

def repair_link(source: str = "ручной запуск") -> bool:
    """Переспаривание + переподключение. Вызывать при подозрении на
    застывший канал. На macOS переспаривание уже встроено в bt_connect."""
    if not repair_lock.acquire(blocking=False):
        log("sys", "Ручное подключение уже выполняется")
        return False
    try:
        return _repair_link(source)
    finally:
        repair_lock.release()


def _repair_link(source: str) -> bool:
    global ser
    if MOCK:
        log("sys", "MOCK-режим: переспаривание не нужно")
        return True
    log("sys", f"Ремонт связи ({source})…")
    with tank_lock:
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        ser = None
        if not IS_MAC:
            if not bt_repair_linux():
                set_link("fail")
                return False
            log("sys", "Переспаривание: пауза перед подключением…")
            time.sleep(5)
        try:
            tank_open()
        except Exception as e:
            if ser is not None:
                ser.close()
                ser = None
            log("sys", f"Ремонт связи: подключение не удалось: {type(e).__name__}: {e}")
            set_link("fail")
            return False
    set_link("ok")
    log("sys", "Связь восстановлена")
    return True

def reset_usb_adapter():
    """Ручной сброс USB-адаптера; связь восстанавливается отдельно."""
    global ser
    if not repair_lock.acquire(blocking=False):
        log("sys", "Операция с Bluetooth уже выполняется")
        return False
    try:
        if MOCK:
            log("sys", "MOCK-режим: сброс USB не выполняется")
            return True
        if IS_MAC:
            log("sys", "Сброс USB-адаптера поддерживается только на Linux")
            return False
        agent_stop.set()
        log("sys", "USB Bluetooth: остановка агента и закрытие соединения с танком")
        with tank_lock:
            if ser is not None:
                try:
                    # Команда выполняется после окончания предыдущей операции.
                    log("tx", "STOP")
                    log("rx", tank_raw("STOP") or "(нет ответа)")
                except Exception as e:
                    log("sys", f"USB Bluetooth: STOP не подтверждён: {type(e).__name__}: {e}")
                finally:
                    ser.close()
                    ser = None
            set_link("fail")
            try:
                log("sys", "USB Bluetooth: сброс адаптера BCM20702A0 (0b05:17cb)")
                result = subprocess.run(USB_RESET_COMMAND, capture_output=True,
                                        text=True, timeout=20)
                output = (result.stdout + result.stderr).strip()
                log("sys", f"usbreset: код {result.returncode}; {output or '(нет вывода)'}")
                # Некоторые версии usbreset возвращают 0 даже при ошибке open/ioctl.
                if result.returncode != 0 or not re.search(r"\bok\s*$", output):
                    raise RuntimeError("USB-сброс не подтверждён; проверьте права и вывод usbreset")
                # После USB reset контроллер может несколько секунд отсутствовать в BlueZ.
                for attempt in range(10):
                    result = subprocess.run(["bluetoothctl", "power", "on"],
                                            capture_output=True, text=True, timeout=10)
                    output = (result.stdout + result.stderr).strip()
                    log("sys", f"bluetoothctl power on (попытка {attempt + 1}): код {result.returncode}; {output or '(нет вывода)'}")
                    if result.returncode == 0 and "succeeded" in output.lower():
                        break
                    if attempt < 9:
                        time.sleep(2)
                else:
                    raise RuntimeError("не удалось включить адаптер после сброса")
                log("sys", "USB Bluetooth: адаптер сброшен. Нажмите «Связь» для подключения танка")
                return True
            except Exception as e:
                log("sys", f"USB Bluetooth: ошибка: {type(e).__name__}: {e}")
                return False
    finally:
        repair_lock.release()

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
                log("sys", f"RFCOMM: ошибка попытки {attempt + 1}: {type(e).__name__}: {e}")
                if attempt < 5:
                    time.sleep(3)
        else:
            raise ConnectionError(f"RFCOMM не поднялся: {last_err}")
    time.sleep(2)
    log("sys", "Порт открыт, проверяю ответ танка")
    log("tx", "PING")
    response = tank_raw("PING", quiet=True)
    log("rx", response or "(нет ответа)")
    if response != "PONG":
        raise ConnectionError(f"PING: ожидался PONG, получено {response!r}")
    # применяем сохранённую калибровку (TRIM/CENTER в танке живут до перезагрузки)
    cal = cal_load()
    if cal:
        l, r = int(cal.get("speedL", 168)), int(cal.get("speedR", 255))
        c = int(cal.get("center", 107))
        log("tx", f"TRIM {l} {r}")
        log("rx", tank_raw(f"TRIM {l} {r}", quiet=True))
        log("tx", f"CENTER {c}")
        log("rx", tank_raw(f"CENTER {c}", quiet=True))
        log("sys", f"Калибровка применена: моторы {l}/{r}, голова {c}°")

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
    if op == "BATT":
        return "BATT:7400"
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
    elif op == "FWD" and response in ("STOPPED_IR", "STOPPED_SONAR"):
        # Остановленное движение прошло неизвестное расстояние; позиция приблизительна.
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
    global ser
    with tank_lock:
        log("sys", f"Команда от {source}")
        log("tx", command)
        if ser is None and not MOCK:
            resp = "(нет связи с танком)"
        else:
            try:
                resp = tank_raw(command)
            except Exception as e:
                resp = f"(ошибка связи: {type(e).__name__}: {e})"
                if ser is not None:
                    ser.close()
                    ser = None
                set_link("fail")
                log("sys", "Связь потеряна; для повторного подключения нажмите «Связь»")
        log("rx", resp or "(молчит)")
    update_pose(command, resp)
    if command == "SCAN":
        record_scan(resp)
    return resp

# ---------------- LLM-агент ----------------

def batt_query():
    """BATT → {"type":"batt","mv":..., "pct":...} или None при ошибке."""
    resp = tank_cmd("BATT", source="ui")
    m = re.search(r"BATT:(\d+)", resp or "")
    if not m:
        return None
    mv = int(m.group(1))
    pct = max(0, min(100, round((mv - BATT_EMPTY_MV) * 100
                                / (BATT_FULL_MV - BATT_EMPTY_MV))))
    return {"type": "batt", "mv": mv, "pct": pct}

def cal_query() -> dict:
    """Текущая калибровка из танка (STATUS)."""
    resp = tank_cmd("STATUS", source="ui")
    cal = {"l": 168, "r": 255, "center": 107}
    m = re.search(r"SPEED:(\d+)/(\d+)", resp)
    if m:
        cal["l"], cal["r"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"CENTER:(\d+)", resp)
    if m:
        cal["center"] = int(m.group(1))
    return cal

def cal_apply(l: int, r: int, center: int):
    """Применить калибровку к танку и сохранить на сервере."""
    ok1 = tank_cmd(f"TRIM {l} {r}", source="ui")
    ok2 = tank_cmd(f"CENTER {center}", source="ui")
    if ok1 == "OK" and ok2 == "OK":
        cal_save({"speedL": l, "speedR": r, "center": center})
        log("sys", f"Калибровка сохранена: моторы {l}/{r}, голова {center}°")
    else:
        log("sys", "Калибровка НЕ применена (танк не подтвердил)")

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
    with log_lock:
        history = list(log_history)
    for event in history:
        await ws.send_text(json.dumps(event, ensure_ascii=False))
    await ws.send_text(json.dumps(
        {"type": "map", "kind": "full", "pose": pose, "trail": trail,
         "obstacles": obstacles}, ensure_ascii=False))
    await ws.send_text(json.dumps({"type": "settings", "heartbeat_s": heartbeat_interval}))
    if link_state:
        await ws.send_text(json.dumps(
            {"type": "link", "state": link_state}, ensure_ascii=False))
    try:
        while True:
            data = json.loads(await ws.receive_text())
            kind = data.get("type")
            if kind == "cmd":                       # ручное управление
                cmd = data.get("cmd", "")
                allowed = {"FWD", "BACK", "LEFT", "RIGHT", "STOP", "SCAN", "IR", "BATT"}
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
            elif kind == "repair":                  # ручное переспаривание
                threading.Thread(target=repair_link, args=("ручной запуск",),
                                 daemon=True).start()
            elif kind == "reset_bluetooth_usb":
                threading.Thread(target=reset_usb_adapter, daemon=True).start()
            elif kind == "set_settings":
                try:
                    set_heartbeat_interval(data.get("heartbeat_s"))
                except (ValueError, OSError) as e:
                    await ws.send_text(json.dumps({"type": "settings_error", "text": str(e)}, ensure_ascii=False))
            elif kind == "get_batt":                # напряжение АКБ
                def _batt():
                    if not BATT_ENABLED:
                        log("sys", "Вольтметр выключен (TANK_BATT=1 на сервере)")
                        return
                    ev = batt_query()
                    if ev:
                        broadcast(ev)
                    else:
                        log("sys", "Батарея: нет ответа (прошивка старая?)")
                threading.Thread(target=_batt, daemon=True).start()
            elif kind == "get_cal":                 # прочитать калибровку из танка
                threading.Thread(
                    target=lambda: broadcast({"type": "cal", **cal_query()}),
                    daemon=True).start()
            elif kind == "set_cal":                 # применить и сохранить
                l = max(0, min(255, int(data.get("l", 168))))
                r = max(0, min(255, int(data.get("r", 255))))
                c = max(5, min(175, int(data.get("center", 107))))
                def _apply():
                    cal_apply(l, r, c)
                    broadcast({"type": "cal", "l": l, "r": r, "center": c})
                threading.Thread(target=_apply, daemon=True).start()
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
    set_link("ok" if MOCK else "fail")
    log("sys", "MOCK-режим" if MOCK else "Автоподключение отключено. Нажмите «Связь» для ручного подключения.")
    threading.Thread(target=heartbeat_loop, daemon=True).start()

def set_link(state: str):
    global link_state
    if state != link_state:
        link_state = state
        broadcast({"type": "link", "state": state})

def heartbeat_loop():
    """Проверка существующего соединения без автоматического восстановления."""
    global ser
    while True:
        if heartbeat_changed.wait(heartbeat_interval):
            heartbeat_changed.clear()
            continue
        if MOCK:
            continue
        with tank_lock:
            if ser is None:
                continue
            try:
                log("tx", "PING (heartbeat)")
                resp = tank_raw("PING")
                log("rx", resp or "(нет ответа)")
                if resp != "PONG":
                    raise ConnectionError(f"PING → {resp!r}")
            except Exception as e:
                log("sys", f"Связь потеряна: {type(e).__name__}: {e}. Нажмите «Связь» для подключения.")
                ser.close()
                ser = None
                set_link("fail")
                continue
        if BATT_ENABLED:
            ev = batt_query()
            if ev:
                broadcast(ev)

if __name__ == "__main__":
    port = 7100
    if "--port" in sys.argv:
        port = int(sys.argv[sys.argv.index("--port") + 1])
    port = int(os.environ.get("PORT", port))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")
