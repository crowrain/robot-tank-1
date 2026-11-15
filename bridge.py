#!/usr/bin/env python3
"""
bridge.py — мост между LLM (OpenAI-совместимый сервер в LAN или локальная Ollama)
и танком по Bluetooth.

Цикл: команда пользователя -> LLM выбирает инструмент -> команда по UART ->
ответ танка (OK / DIST:...) возвращается в контекст -> LLM продолжает,
пока не даст финальный текстовый ответ.

Зависимости: pip install pyserial
Запуск:      python bridge.py
Настройки — вверху файла (порт, модель).
"""

import json
import os
import sys
import urllib.request

import serial

# ---------------- Настройки ----------------
# Все адреса, ключи и пины — ТОЛЬКО через переменные окружения.
# Их можно задать в окружении или в файле .env рядом с bridge.py
# (формат KEY=VALUE, .env не коммитится — см. .gitignore и .env.example).

def _load_env_file():
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if not os.path.exists(env_path):
        return
    with open(env_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))

_load_env_file()

def _require(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError(
            f"Не задана переменная окружения {name}. "
            f"Скопируйте .env.example в .env и заполните значения.")
    return value

# macOS: /dev/tty.HC-06 или /dev/tty.HC-06-DevB (смотрите: ls /dev/tty.*)
# Windows: COM3, COM5... (Диспетчер устройств -> Порты)
# Linux:   /dev/rfcomm0 (после привязки через bluetoothctl/rfcomm)
SERIAL_PORT = os.environ.get("TANK_SERIAL_PORT", "/dev/cu.HC-06")
BT_ADDR = _require("TANK_BT_ADDR")        # MAC модуля HC-06
BT_PIN = os.environ.get("TANK_BT_PIN", "1234")
BAUD = int(os.environ.get("TANK_BAUD", "9600"))

# Бэкенд LLM: "openai" (llama-swap в LAN) или "ollama" (локально)
LLM_BACKEND = os.environ.get("TANK_LLM_BACKEND", "openai")

# --- OpenAI-совместимый сервер (llama-swap) ---
MODEL = os.environ.get("TANK_MODEL", "Qwen3.8-FN-IQ3-S-Basalt")
OPENAI_URL = _require("TANK_OPENAI_URL")   # http://<llm-сервер>:8080/v1/chat/completions
OPENAI_KEY = _require("TANK_OPENAI_KEY")

# --- локальная Ollama (запасной вариант) ---
OLLAMA_URL = os.environ.get("TANK_OLLAMA_URL", "http://localhost:11434/api/chat")

SERIAL_TIMEOUT_S = 12.0   # сканирование датчиком занимает ~3 с; запас большой

# ---------------- Инструменты для LLM ----------------

TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "move",
            "description": "Двигать танк. duration_ms — длительность в миллисекундах, "
                           "максимум 3000. Ориентиры: разворот на 90° ≈ 500-700 мс "
                           "(подбирается экспериментально).",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "left", "right"],
                        "description": "forward — езда вперёд, left/right — разворот на месте. "
                                       "Движение назад ЗАПРЕЩЕНО.",
                    },
                    "duration_ms": {
                        "type": "integer",
                        "minimum": 100,
                        "maximum": 3000,
                    },
                },
                "required": ["direction", "duration_ms"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "scan",
            "description": "Замерить дистанцию до препятствий ультразвуковым датчиком "
                           "в 5 направлениях: крайне влево, влево, прямо, вправо, крайне вправо. "
                           "Возвращает расстояния в сантиметрах; 400 = свободно.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "check_ir",
            "description": "Проверить инфракрасные датчики препятствий спереди "
                           "(смотрят на 45° влево и вправо). Видят тонкие препятствия "
                           "вроде ножек мебели, которые пропускает ультразвук.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop",
            "description": "Немедленно остановить моторы.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
]

SYSTEM_PROMPT = (
    "Ты управляешь учебным роботом-танком через инструменты. "
    "Инструмент scan возвращает расстояния до препятствий в сантиметрах: "
    "ЧЕМ БОЛЬШЕ число, ТЕМ БОЛЬШЕ свободного места в этом направлении; "
    "меньше 15 см впереди — уже тесно, туда не ездить; 15 см и больше — можно ехать. "
    "Перед каждым движением вперёд вызывай scan и езжай, только если свободно. "
    "Длительность одного движения не больше 3000 мс. "
    "ДВИЖЕНИЕ НАЗАД ЗАПРЕЩЕНО В ЛЮБОЙ СИТУАЦИИ. "
    "Если впереди тесно — ищи проезд так: крутись на месте ТОЛЬКО ВЛЕВО "
    "маленькими шагами по 250-300 мс (это примерно 30-40°), после каждого поворота "
    "вызывай scan; как только впереди свободно 15+ см — езжай вперёд. "
    "Спереди стоят ИК-датчики (check_ir): они видят то, что пропускает ультразвук "
    "(ножки мебели), и прошивка сама тормозит при препятствии впереди — если move "
    "вернул «Движение ПРЕРВАНО», не повторяй движение в ту же сторону, крутись. "
    "Отвечай пользователю коротко по-русски: что сделал и что показали датчики."
)

# ---------------- Serial ----------------

class Tank:
    def __init__(self, port: str, baud: int, bt_addr: str = None):
        # macOS рвёт SPP-линк при закрытии порта и не всегда поднимает его
        # при открытии — просим систему подключиться явно (blueutil)
        if bt_addr:
            import subprocess
            import time as _t

            def bt(*args):
                return subprocess.run(["blueutil", *args],
                                      capture_output=True, text=True)

            # Клоны HC-06 + macOS: данные идут только по "свежему" каналу,
            # поэтому перед каждым сеансом делаем переспаривание.
            print("[bt] переспаривание HC-06...")
            bt("--disconnect", bt_addr)
            bt("--unpair", bt_addr)
            _t.sleep(3)
            bt("--pair", bt_addr, BT_PIN)
            _t.sleep(2)
            bt("--connect", bt_addr)
            for _ in range(20):                    # ждём реального линка до 10 с
                if bt("--is-connected", bt_addr).stdout.strip() == "1":
                    print("[bt] соединено")
                    break
                _t.sleep(0.5)
            else:
                raise ConnectionError(
                    "Не удалось подключиться к HC-06. Проверьте, что танк "
                    "включён и модуль мигает (ждёт соединения).")
            import os
            for _ in range(20):                    # ждём появления /dev-порта
                if os.path.exists(port):
                    break
                _t.sleep(0.5)
            else:
                raise ConnectionError(f"Порт {port} не появился после подключения.")
        self.ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT_S)
        print(f"[serial] порт {port} открыт, жду сброс Arduino...")
        import time
        time.sleep(2)                      # Arduino перезагружается при открытии порта
        self.ser.reset_input_buffer()
        greeting = self.ser.readline().decode(errors="replace").strip()
        print(f"[serial] приветствие: {greeting}")
        # проверка связи: PING с несколькими попытками
        for attempt in range(3):
            resp = self.cmd("PING")
            if resp == "PONG":
                return
            time.sleep(1)
        raise ConnectionError(
            "Танк не ответил на PING. Проверьте: питание включено, "
            "светодиод HC-06 горит ровно (не мигает).")

    def cmd(self, command: str) -> str:
        self.ser.reset_input_buffer()
        self.ser.write((command + "\n").encode())
        resp = self.ser.readline().decode(errors="replace").strip()
        print(f"[tank] {command} -> {resp}")
        return resp

# ---------------- Выполнение инструментов ----------------

def run_tool(tank: Tank, name: str, args: dict) -> str:
    try:
        if name == "move":
            direction = args.get("direction", "forward")
            if direction == "backward":
                return ("Движение назад ЗАПРЕЩЕНО правилами. Крутись на месте "
                        "(left/right) и вызывай scan, пока не найдётся проезд вперёд.")
            duration = int(args.get("duration_ms", 500))
            duration = max(100, min(3000, duration))     # валидация от галлюцинаций
            op = {"forward": "FWD", "backward": "BACK",
                  "left": "LEFT", "right": "RIGHT"}[direction]
            resp = tank.cmd(f"{op} {duration}")
            if resp == "STOPPED_IR":
                return ("Движение ПРЕРВАНО: инфракрасный датчик увидел препятствие "
                        "спереди (возможно, ножка мебели, которую не видит ультразвук). "
                        "Крутись на месте и ищи проезд вперёд (назад ездить запрещено).")
            if resp == "STOPPED_SONAR":
                return ("Движение ПРЕРВАНО: ультразвуковой датчик обнаружил "
                        "препятствие впереди. Не двигайся вперёд; осмотрись.")
            if resp == "STOPPED_SONAR_NO_ECHO":
                return ("Движение ПРЕРВАНО: ультразвуковой датчик не дал эха. "
                        "Проезд не подтверждён; не продолжай движение вперёд.")
            return resp
        if name == "scan":
            resp = tank.cmd("SCAN")
            if resp.startswith("DIST:"):
                vals = [v.strip() for v in resp[5:].split(",")]
                labels = ["far_left", "left", "forward", "right", "far_right"]
                data = dict(zip(labels, vals))
                return ("Дистанция до препятствия в см по направлениям: "
                        + json.dumps(data, ensure_ascii=False)
                        + ". 400 = свободно.")
            return f"Ошибка танка: {resp}"
        if name == "check_ir":
            resp = tank.cmd("IR")
            if resp.startswith("IR:"):
                l, r = resp[3:].split(",")
                return ("ИК-датчики спереди (смотрят на 45° влево/вправо, видят "
                        "то, что не видит ультразвук — например ножки мебели): "
                        + json.dumps({"left_obstacle": l == "1",
                                      "right_obstacle": r == "1"},
                                     ensure_ascii=False))
            return f"Ошибка танка: {resp}"
        if name == "stop":
            return tank.cmd("STOP")
        return f"Неизвестный инструмент: {name}"
    except Exception as e:                     # не роняем диалог из-за сбоя связи
        return f"Ошибка связи с танком: {e}"

# ---------------- LLM ----------------

def _post(url, body, headers=None, timeout=300):
    h = {"Content-Type": "application/json"}
    if headers:
        h.update(headers)
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers=h)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())

def chat(messages):
    """Возвращает нормализованное сообщение: content, tool_calls[{id, function{name, arguments(dict)}}]."""
    if LLM_BACKEND == "openai":
        body = {"model": MODEL, "messages": messages, "tools": TOOLS,
                "temperature": 0.2, "stream": False}
        d = _post(OPENAI_URL, body,
                  headers={"Authorization": f"Bearer {OPENAI_KEY}"})
        msg = d["choices"][0]["message"]
        for tc in msg.get("tool_calls") or []:
            args = tc["function"].get("arguments")
            if isinstance(args, str):          # OpenAI шлёт аргументы строкой JSON
                try:
                    tc["function"]["arguments"] = json.loads(args) if args else {}
                except json.JSONDecodeError:
                    tc["function"]["arguments"] = {}
        return msg

    # ollama
    body = {"model": MODEL, "messages": messages, "tools": TOOLS,
            "stream": False, "options": {"temperature": 0.2}}
    return _post(OLLAMA_URL, body, timeout=180)["message"]

# ---------------- Главный цикл ----------------

def main():
    try:
        tank = Tank(SERIAL_PORT, BAUD, bt_addr=BT_ADDR)
    except Exception as e:
        print(f"Не удалось открыть порт {SERIAL_PORT}: {e}")
        print("Проверьте: HC-06 спарен с компьютером, порт указан правильно, "
              "танк включён.")
        sys.exit(1)

    messages = [{"role": "system", "content": SYSTEM_PROMPT}]
    print(f"Готово. Модель: {MODEL}. Пишите команду (пустая строка — выход).")

    while True:
        try:
            user = input("\nВы > ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not user:
            break

        messages.append({"role": "user", "content": user})

        for _ in range(40):                        # цикл инструментов, с предохранителем
            try:
                msg = chat(messages)
            except Exception as e:
                print(f"Ошибка LLM: {e}")
                break

            messages.append(msg)
            tool_calls = msg.get("tool_calls") or []
            if not tool_calls:
                print(f"Танк > {msg.get('content', '').strip()}")
                break

            for tc in tool_calls:
                fn = tc["function"]
                result = run_tool(tank, fn["name"], fn.get("arguments") or {})
                tool_msg = {"role": "tool", "name": fn["name"], "content": result}
                if tc.get("id"):               # OpenAI-формат требует tool_call_id
                    tool_msg["tool_call_id"] = tc["id"]
                messages.append(tool_msg)
        else:
            tank.cmd("STOP")
            print("Танк > Превышен лимит шагов, остановился на всякий случай.")

    tank.cmd("STOP")
    tank.ser.close()
    print("Порт закрыт, моторы остановлены. Пока!")

if __name__ == "__main__":
    main()
