#!/usr/bin/env python3
"""Claude-driven bridge for the tiny_quad_dog quadruped.

Translates natural-language commands into calls on the ESP32's serial JSON
command protocol (see include/command_interface.h) using Claude's tool
calling via the Anthropic Python SDK's tool runner. This is the reference
implementation of "an API, e.g. Claude, controls the robot through the
abstraction layer" described in the project README: the firmware only ever
sees {"function": ..., "args": {...}} commands -- swapping this script for a
different LLM, transport, or a hand-written controller requires zero
firmware changes.

Usage:
    export ANTHROPIC_API_KEY=...   # or `ant auth login`
    pip install -r tools/requirements.txt
    python tools/claude_bridge.py --port /dev/tty.usbserial-XXXX

On Windows, --port is a COM port (e.g. COM5); on Linux, typically
/dev/ttyUSB0 or /dev/ttyACM0.
"""
from __future__ import annotations

import argparse
import json
import threading
import time

import serial  # pyserial
from anthropic import Anthropic, beta_tool

# Per-request override: pass --model claude-sonnet-5 or claude-haiku-4-5 for
# cheaper/faster interactive control if Opus-tier latency/cost isn't needed
# for your use case.
DEFAULT_MODEL = "claude-opus-5"
DEFAULT_BAUD = 115200
COMMAND_TIMEOUT_S = 5.0

SYSTEM_PROMPT = (
    "You are the control interface for a small 4-legged walking robot "
    '("tiny_quad_dog"). Use the provided tools to move, turn, pose, stop, '
    "or change the robot's gait speed in response to the user's request. "
    "Prefer small, safe motions; ask for clarification only if the request "
    "is ambiguous about direction or could be unsafe. After acting, briefly "
    "confirm what the robot did."
)


class RobotLink:
    """Serial JSON-line transport to the ESP32 CommandInterface.

    One line of JSON out, one line of JSON back -- see the protocol
    documented at the top of include/command_interface.h. Kept intentionally
    dumb: all the "what should the robot do" reasoning lives in Claude, not
    here, so this class would be identical for any other LLM or controller.
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        self._ser = serial.Serial(port, baud, timeout=COMMAND_TIMEOUT_S)
        self._lock = threading.Lock()
        # Opening the port resets the ESP32; give it time to finish booting
        # and printing its startup banner before we start sending commands.
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


def build_tools(link: RobotLink):
    """One tool per RobotApi method (src/robot_api.cpp is the source of
    truth for the command surface -- keep this list in sync with it)."""

    @beta_tool
    def move(direction: str, speed: float = 0.5, duration_ms: int = 1000) -> str:
        """Walk the robot in a direction for a fixed duration.

        Args:
            direction: One of "forward", "backward", "left", "right".
            speed: Gait speed multiplier, 0.0-2.0 (1.0 is the default pace).
            duration_ms: How long this motion runs before another command is
                needed to continue or change it, 100-10000.
        """
        return json.dumps(link.send("move", {"direction": direction, "speed": speed, "duration_ms": duration_ms}))

    @beta_tool
    def turn(direction: str, angle_deg: float = 30.0) -> str:
        """Turn the robot in place.

        Args:
            direction: "left" or "right".
            angle_deg: Approximate turn angle, -90 to 90.
        """
        return json.dumps(link.send("turn", {"direction": direction, "angle_deg": angle_deg}))

    @beta_tool
    def set_pose(pose: str) -> str:
        """Move the robot into a named static pose.

        Args:
            pose: One of "sit", "stand", "idle".
        """
        return json.dumps(link.send("setPose", {"pose": pose}))

    @beta_tool
    def stop() -> str:
        """Immediately stop all motion and settle into the idle stance."""
        return json.dumps(link.send("stop"))

    @beta_tool
    def set_gait_speed(speed: float) -> str:
        """Change the walking cadence used by subsequent move() calls.

        Args:
            speed: 0.1-2.0; higher is faster stepping.
        """
        return json.dumps(link.send("setGaitSpeed", {"speed": speed}))

    return [move, turn, set_pose, stop, set_gait_speed]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Serial port for the ESP32, e.g. /dev/tty.usbserial-XXXX or COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    args = parser.parse_args()

    link = RobotLink(args.port, args.baud)
    print(f"[robot] {link.send('status')}")

    client = Anthropic()  # reads ANTHROPIC_API_KEY, or an `ant auth login` profile
    tools = build_tools(link)
    messages: list[dict] = []

    print("Type a command for the robot (e.g. 'walk forward for a second, then sit'). Ctrl-C to exit.")
    try:
        while True:
            user_input = input("> ").strip()
            if not user_input:
                continue
            messages.append({"role": "user", "content": user_input})

            runner = client.beta.messages.tool_runner(
                model=args.model,
                max_tokens=1024,
                system=SYSTEM_PROMPT,
                tools=tools,
                messages=messages,
            )
            final_message = None
            for message in runner:
                final_message = message
            if final_message is not None:
                # Keep only the final text turn in history (not the
                # intermediate tool_use/tool_result exchange) -- enough for
                # coherent multi-turn conversation without replaying every
                # tool call back to the model on each new request.
                messages.append({"role": "assistant", "content": final_message.content})
                for block in final_message.content:
                    if block.type == "text":
                        print(block.text)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
