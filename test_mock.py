#!/usr/bin/env python3
"""
test_mock.py — проверка связки Ollama + function calling без железа.
Имитирует танк: команды логируются, SCAN возвращает фиктивные дистанции.
Запуск: python test_mock.py
"""

import bridge


class MockTank:
    def cmd(self, command: str) -> str:
        if command == "SCAN":
            return "DIST:120,80,35,90,150"
        if command == "PING":
            return "PONG"
        return "OK"


def main():
    tank = MockTank()
    messages = [{"role": "system", "content": bridge.SYSTEM_PROMPT}]
    user = "Осмотрись и скажи, куда безопасно проехать. Если можно — проедь вперёд одну секунду."
    messages.append({"role": "user", "content": user})
    print("Запрос:", user, "\n")

    for _ in range(10):
        msg = bridge.chat(messages)
        messages.append(msg)
        tool_calls = msg.get("tool_calls") or []
        if not tool_calls:
            print("LLM (финал):", msg.get("content", "").strip())
            return
        for tc in tool_calls:
            fn = tc["function"]
            result = bridge.run_tool(tank, fn["name"], fn.get("arguments") or {})
            print(f"  tool: {fn['name']}({fn.get('arguments')}) -> {result}")
            messages.append({"role": "tool", "name": fn["name"], "content": result})
    print("Превышен лимит итераций")


if __name__ == "__main__":
    main()
