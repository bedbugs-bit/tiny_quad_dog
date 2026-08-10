#!/usr/bin/env python3
"""OpenAI function-calling bridge for tiny_quad_dog.

Translate natural language into JSON commands sent over serial to the
ESP32 CommandInterface using OpenAI Chat completions with function calling.

Usage:
    export OPENAI_API_KEY=...
    pip install -r tools/requirements.txt
    python tools/openai_bridge.py --port /dev/tty.usbserial-XXXX

This implements the same transport and JSON command format used by
`tools/claude_bridge.py` so firmware does not need to change.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
import os

import serial
import openai

DEFAULT_BAUD = 115200
COMMAND_TIMEOUT_S = 5.0

SYSTEM_PROMPT = (
    "You are the control interface for a small 4-legged walking robot (\"tiny_quad_dog\"). "
    "Reply by selecting and calling one of the provided functions with appropriate arguments. "
    "Prefer small, safe motions and ask for clarification if a request is ambiguous or unsafe."
)


class RobotLink:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        self._ser = serial.Serial(port, baud, timeout=COMMAND_TIMEOUT_S)
        self._lock = threading.Lock()
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    def send(self, function: str, args: dict | None = None) -> dict:
        payload = {"function": function, "args": args or {}}
        line = json.dumps(payload) + "\n"
        with self._lock:
            self._ser.write(line.encode("utf-8"))
            self._ser.flush()
            raw = self._ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            return {"status": "error", "message": "no response from robot (serial timeout)"}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"status": "error", "message": f"unparseable response: {raw!r}"}

    def close(self):
        self._ser.close()


FUNCTIONS = [
    {
        "name": "move",
        "description": "Walk the robot in a direction for a fixed duration.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["forward", "backward", "left", "right"]},
                "speed": {"type": "number", "minimum": 0.0, "maximum": 2.0},
                "duration_ms": {"type": "integer", "minimum": 100, "maximum": 10000},
            },
            "required": ["direction"],
        },
    },
    {
        "name": "turn",
        "description": "Turn the robot in place.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["left", "right"]},
                "angle_deg": {"type": "number", "minimum": -90.0, "maximum": 90.0},
            },
            "required": ["direction"],
        },
    },
    {
        "name": "setPose",
        "description": "Set a named static pose.",
        "parameters": {
            "type": "object",
            "properties": {
                "pose": {"type": "string", "enum": ["sit", "stand", "idle"]},
            },
            "required": ["pose"],
        },
    },
    {
        "name": "stop",
        "description": "Stop all motion.",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "setGaitSpeed",
        "description": "Change walking cadence.",
        "parameters": {
            "type": "object",
            "properties": {"speed": {"type": "number", "minimum": 0.1, "maximum": 2.0}},
            "required": ["speed"],
        },
    },
]


def call_robot_from_function(link: RobotLink, name: str, arguments: dict) -> dict:
    # Map chat function name to transport function name used by firmware
    mapping = {
        "move": "move",
        "turn": "turn",
        "setPose": "setPose",
        "stop": "stop",
        "setGaitSpeed": "setGaitSpeed",
    }
    func = mapping.get(name)
    if func is None:
        return {"status": "error", "message": f"unknown function {name}"}
    return link.send(func, arguments)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    key = os.environ.get("OPENAI_API_KEY")
    if not key:
        print("Please set OPENAI_API_KEY in your environment.")
        return
    openai.api_key = key

    link = RobotLink(args.port, args.baud)
    print(f"[robot] {link.send('status')}")

    messages: list[dict] = []
    print("Type a command for the robot. Ctrl-C to exit.")
    try:
        while True:
            user_input = input("> ").strip()
            if not user_input:
                continue
            messages.append({"role": "user", "content": user_input})

            response = openai.ChatCompletion.create(
                model="gpt-4o-mini",  # change to preferred model supporting functions
                messages=[{"role": "system", "content": SYSTEM_PROMPT}] + messages,
                functions=FUNCTIONS,
                function_call="auto",
                max_tokens=512,
            )

            choice = response["choices"][0]
            message = choice["message"]
            if message.get("function_call"):
                fname = message["function_call"]["name"]
                raw_args = message["function_call"].get("arguments") or "{}"
                try:
                    args_obj = json.loads(raw_args) if isinstance(raw_args, str) else raw_args
                except Exception:
                    args_obj = {}
                result = call_robot_from_function(link, fname, args_obj)
                print("-> robot reply:", result)
                # append assistant textual summary if provided
                messages.append({"role": "assistant", "content": f"called {fname} with {args_obj}"})
            else:
                # fallback: plain text reply from model
                text = message.get("content", "")
                print(text)
                messages.append({"role": "assistant", "content": text})
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
