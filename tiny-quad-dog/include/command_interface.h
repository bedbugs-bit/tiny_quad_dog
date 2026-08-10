#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "robot_api.h"

#ifndef TQD_ENABLE_WIFI_SERVER
#define TQD_ENABLE_WIFI_SERVER 0
#endif

#if TQD_ENABLE_WIFI_SERVER
#include <WebServer.h>
#include <WiFi.h>
#endif

// CommandInterface is the abstraction boundary described in the project
// README: the ESP32 only understands a small, transport-agnostic JSON
// command protocol and never talks to an LLM API directly. Whatever is
// deciding what the robot should do next -- a hand-typed command, a
// scripted routine, or a companion process driving Claude's tool-calling
// (see tools/claude_bridge.py) -- speaks this same protocol.
//
// Request  (one line of JSON over Serial, or an HTTP POST body):
//   {"id": "optional-caller-supplied-id",
//    "function": "move" | "turn" | "setPose" | "stop" | "setGaitSpeed" | "status",
//    "args": { ... function-specific ... }}
//
// Response (mirrors the id so callers can match async replies):
//   {"id": "...", "status": "ok" | "error", "message": "..."}
//
// Function reference:
//   move        {direction: "forward"|"backward"|"left"|"right", speed: 0..2, duration_ms: 100..10000}
//   turn        {direction: "left"|"right", angle_deg: -90..90}
//   setPose     {pose: "sit"|"stand"|"idle"}
//   stop        {}
//   setGaitSpeed{speed: 0.1..2}
//   status      {}  -> reply includes firmware/uptime info, no robot motion
class CommandInterface {
public:
  CommandInterface();

  // Serial is always available (no network setup required) and is the
  // primary/expected transport for the companion bridge script.
  void begin(RobotApi* robotApi);

  // Optional: also serve the same protocol over a local HTTP endpoint so a
  // companion running elsewhere on the LAN can control the robot wirelessly.
  // Only compiled in when TQD_ENABLE_WIFI_SERVER=1 (see platformio.ini) so a
  // pure-serial setup never needs Wi-Fi credentials or pulls in networking
  // code it isn't using.
  void beginWifi(const char* ssid, const char* password, uint16_t port = 80);

  // Non-blocking: call every loop() iteration. Never sleeps/blocks, so it is
  // always safe to interleave with servoDriver.update() and gaitEngine.update().
  void update();

private:
  RobotApi* robotApi_ = nullptr;
  String serialLineBuffer_;
  unsigned long beginMs_ = 0;

#if TQD_ENABLE_WIFI_SERVER
  WebServer* server_ = nullptr;
  void handleHttpCommand();
  void handleHttpStatus();
#endif

  // Parses one JSON command line/body, dispatches to RobotApi, and returns
  // the serialized JSON response. Shared by both the Serial and HTTP paths
  // so the protocol behaves identically regardless of transport.
  String processCommand(const String& payload);
  bool dispatch(const String& functionName, JsonObjectConst args, String& messageOut);
};
