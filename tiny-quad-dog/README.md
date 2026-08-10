# Tiny Quad Dog

ESP32 firmware for a small 4-legged walking robot, plus a Python companion
script that lets Claude (or any other LLM/controller) drive it over a small,
transport-agnostic JSON command protocol.

## Architecture

```
tools/claude_bridge.py                      ESP32 firmware
  - talks to the Claude API (tool calling)     CommandInterface   (parses JSON, Serial/Wi-Fi)
  - translates each tool call into a               |
    JSON robot command                             v
                                                  RobotApi          (move / turn / setPose / stop / setGaitSpeed)
        ---- JSON over USB serial ---->              |
        {"function":"move","args":{...}}             v
        <---- {"status":"ok",...} ----             GaitEngine        (trot gait -> per-leg foot trajectories)
                                                      |
                                                      v
                                                  LegKinematics       (per-leg inverse/forward kinematics)
                                                      |
                                                      v
                                                  ServoDriver         (PCA9685 over I2C -> servo pulses)
```

The ESP32 **never talks to an LLM API directly** — no Wi-Fi credentials, no
API keys, no TLS stack on the microcontroller. It only understands a small
JSON command protocol (`CommandInterface`, documented in
[include/command_interface.h](include/command_interface.h)). Whatever decides
*what* the robot should do next — a person typing commands, a scripted
routine, or a companion process driving Claude's tool-calling — speaks that
same protocol. This keeps the two concerns cleanly separated: the firmware
owns "how do I walk," the companion owns "what should I do."

Layers, bottom to top:

| Layer | File | Responsibility |
|---|---|---|
| `ServoDriver` | [servo_driver.h/.cpp](include/servo_driver.h) | Talks to the PCA9685 PWM driver over I2C; converts angles to pulse widths; smoothly interpolates each channel. |
| `LegKinematics` | [leg_kinematics.h/.cpp](include/leg_kinematics.h) | Analytic 2-link inverse/forward kinematics per leg, plus per-joint calibration (offset/invert/clamp). Pure math — no hardware dependency, unit tested on the host (see Testing below). |
| `GaitEngine` | [gait_engine.h/.cpp](include/gait_engine.h) | Turns high-level intents (walk, turn, sit, stand, wag) into per-leg foot trajectories over time, using a trot gait (diagonal leg pairs step together). |
| `RobotApi` | [robot_api.h/.cpp](include/robot_api.h) | The stable, minimal command surface: `move`, `turn`, `setPose`, `stop`, `setGaitSpeed`. |
| `CommandInterface` | [command_interface.h/.cpp](include/command_interface.h) | Parses JSON commands (Serial, and optionally local Wi-Fi HTTP) and dispatches to `RobotApi`. This is the abstraction boundary an LLM controller talks to. |

## Building and flashing

```bash
cd tiny-quad-dog
pio run                    # build
pio run -t upload          # flash over USB
pio device monitor -b 115200
```

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`, or use the bundled `.venv` if present).

## Testing

The pure-math kinematics module (`LegKinematics`) has no Arduino/ESP32
dependency by design, so it's unit tested on the host — no hardware needed:

```bash
pio test -e native
```

This is real coverage, not a formality: it caught a sign error in the
inverse-kinematics law-of-cosines term during development (the two-link IK
was solving for the wrong elbow angle on most leg poses). Run it after any
change to `leg_kinematics.cpp`.

## Controlling the robot

### Over serial (always available)

Once flashed, the robot accepts line-delimited JSON commands over USB serial
at 115200 baud:

```jsonc
// → {"id": "1", "function": "move", "args": {"direction": "forward", "speed": 1.0, "duration_ms": 1000}}
// ← {"id": "1", "status": "ok"}
```

Function reference:

| Function | Args | Notes |
|---|---|---|
| `move` | `direction` (forward/backward/left/right), `speed` (0-2), `duration_ms` (100-10000) | Starts a trot gait in that direction. |
| `turn` | `direction` (left/right), `angle_deg` (-90 to 90) | Turns in place. |
| `setPose` | `pose` (sit/stand/idle) | Moves to a static pose. |
| `stop` | — | Returns to idle immediately. |
| `setGaitSpeed` | `speed` (0.1-2) | Changes step cadence for future `move`/`turn` calls. |
| `status` | — | Health check; no motion. |

You can drive this by hand with any serial terminal (`pio device monitor`,
`screen`, `minicom`, ...) — it's plain JSON, one line in, one line out.

### Optional: over local Wi-Fi

Build with `-D TQD_ENABLE_WIFI_SERVER=1` (see `platformio.ini`) and call
`commandInterface.beginWifi(ssid, password)` in `main.cpp` to also serve the
same protocol over HTTP (`POST /command`, `GET /status`) on the local
network. Off by default so a pure-USB setup needs no Wi-Fi credentials.

### With Claude

```bash
cd tools
pip install -r requirements.txt
export ANTHROPIC_API_KEY=...      # or `ant auth login`
python claude_bridge.py --port /dev/tty.usbserial-XXXX
```

Then just type what you want the robot to do:

```
> walk forward for a couple seconds, then sit down
```

`claude_bridge.py` defines one tool per `RobotApi` method and uses the
Anthropic Python SDK's tool runner to let Claude decide which to call; each
tool call is translated 1:1 into the JSON serial protocol above. Swapping in
a different model, a different LLM provider, or a hand-written rule-based
controller means editing this one script — the firmware doesn't change.

### With OpenAI / function-calling (Codex / ChatGPT)

An alternative bridge using OpenAI function-calling is provided at
`tools/openai_bridge.py`. It implements the same JSON command protocol and
can be used with OpenAI models that support function calls. Example usage:

```bash
cd tools
pip install -r requirements.txt
export OPENAI_API_KEY=...
python openai_bridge.py --port /dev/tty.usbserial-XXXX
```

The script requests the model to select one of the robot functions and then
translates that function call into the same serial JSON command the
firmware expects.


## Suggested test sequence

1. Flash the firmware and confirm the PCA9685 responds on I2C (watch the
   Serial monitor for I2C errors at boot).
2. Send `{"function":"setPose","args":{"pose":"idle"}}` so all servos settle
   into a neutral pose (fully retracted — safe to power up on a bench without
   the legs fighting the mounting).
3. Test each motion individually before combining them:
   `{"function":"setPose","args":{"pose":"stand"}}`, then `move` in each of
   the four directions, `turn` left/right, `setPose: sit`, and
   `setGaitSpeed`.
4. Only after the mechanics are verified on hardware should you connect the
   Claude bridge and hand control to natural-language commands.
