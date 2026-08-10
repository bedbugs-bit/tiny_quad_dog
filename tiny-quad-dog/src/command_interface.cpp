#include "command_interface.h"

namespace {
constexpr size_t kMaxLineLength = 512;

String buildResponse(const String& id, bool ok, const String& message) {
  JsonDocument doc;
  if (id.length() > 0) {
    doc["id"] = id;
  }
  doc["status"] = ok ? "ok" : "error";
  if (message.length() > 0) {
    doc["message"] = message;
  }
  String out;
  serializeJson(doc, out);
  return out;
}
}  // namespace

static bool equalsIgnoreCase(const String& a, const char* b) {
  return strcasecmp(a.c_str(), b) == 0;
}

static String processStatusResponse(unsigned long beginMs, RobotApi* robotApi, const String& id) {
  StaticJsonDocument<256> doc;
  if (id.length() > 0) {
    doc["id"] = id;
  }
  doc["status"] = "ok";
  JsonObject info = doc.createNestedObject("info");
  info["firmware"] = "tiny_quad_dog";
  info["uptime_ms"] = millis() - beginMs;
  info["robot_api_initialized"] = (bool)(robotApi != nullptr);
  if (robotApi != nullptr) {
    info["robot_status"] = robotApi->getStatusString();
  }
  String out;
  serializeJson(doc, out);
  return out;
}

CommandInterface::CommandInterface() {}

void CommandInterface::begin(RobotApi* robotApi) {
  robotApi_ = robotApi;
  beginMs_ = millis();
  serialLineBuffer_.reserve(kMaxLineLength);
}

void CommandInterface::beginWifi(const char* ssid, const char* password, uint16_t port) {
#if TQD_ENABLE_WIFI_SERVER
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[command_interface] Wi-Fi connect failed; HTTP server not started.");
    return;
  }

  server_ = new WebServer(port);
  server_->on("/command", HTTP_POST, [this]() { handleHttpCommand(); });
  server_->on("/status", HTTP_GET, [this]() { handleHttpStatus(); });
  server_->begin();
  Serial.print("[command_interface] HTTP command server listening at http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(port);
#else
  (void)ssid;
  (void)password;
  (void)port;
  Serial.println("[command_interface] Wi-Fi server disabled at build time (TQD_ENABLE_WIFI_SERVER=0).");
#endif
}

void CommandInterface::update() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      serialLineBuffer_.trim();
      if (serialLineBuffer_.length() > 0) {
        Serial.println(processCommand(serialLineBuffer_));
      }
      serialLineBuffer_ = "";
    } else if (c != '\r' && serialLineBuffer_.length() < kMaxLineLength) {
      serialLineBuffer_ += c;
    }
  }

#if TQD_ENABLE_WIFI_SERVER
  if (server_ != nullptr) {
    server_->handleClient();
  }
#endif
}

#if TQD_ENABLE_WIFI_SERVER
void CommandInterface::handleHttpCommand() {
  if (server_ == nullptr) {
    return;
  }
  const String body = server_->arg("plain");
  const String response = processCommand(body);
  server_->send(200, "application/json", response);
}

void CommandInterface::handleHttpStatus() {
  if (server_ == nullptr) {
    return;
  }
  server_->send(200, "application/json", processCommand("{\"function\":\"status\"}"));
}
#endif

String CommandInterface::processCommand(const String& payload) {
  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return buildResponse("", false, String("invalid JSON: ") + error.c_str());
  }

  const String id = doc["id"] | "";
  const String functionName = doc["function"] | "";
  if (functionName.length() == 0) {
    return buildResponse(id, false, "missing \"function\"");
  }

  // Special-case status to return richer telemetry
  if (functionName == "status") {
    return processStatusResponse(beginMs_, robotApi_, id);
  }

  JsonObjectConst args = doc["args"];
  String message;
  const bool ok = dispatch(functionName, args, message);
  return buildResponse(id, ok, message);
}

bool CommandInterface::dispatch(const String& functionName, JsonObjectConst args, String& messageOut) {
  // status handled specially in processCommand for richer info

  if (robotApi_ == nullptr) {
    messageOut = "robot API not initialized";
    return false;
  }

  if (functionName == "move") {
    const String direction = args["direction"] | "";
    if (direction.length() == 0 || !(equalsIgnoreCase(direction, "forward") || equalsIgnoreCase(direction, "backward") || equalsIgnoreCase(direction, "left") || equalsIgnoreCase(direction, "right"))) {
      messageOut = "invalid direction for move; expected forward/backward/left/right";
      return false;
    }
    const float speed = args["speed"] | 0.5f;
    if (speed < 0.0f || speed > 2.0f) {
      messageOut = "invalid speed; expected 0.0-2.0";
      return false;
    }
    const int durationMs = args["duration_ms"] | 1000;
    if (durationMs < 100 || durationMs > 10000) {
      messageOut = "invalid duration_ms; expected 100-10000";
      return false;
    }
    robotApi_->move(direction.c_str(), speed, durationMs);
    return true;
  }

  if (functionName == "turn") {
    const String direction = args["direction"] | "";
    if (direction.length() == 0 || !(equalsIgnoreCase(direction, "left") || equalsIgnoreCase(direction, "right"))) {
      messageOut = "invalid direction for turn; expected left/right";
      return false;
    }
    const float angleDeg = args["angle_deg"] | 30.0f;
    if (angleDeg < -90.0f || angleDeg > 90.0f) {
      messageOut = "invalid angle_deg; expected -90..90";
      return false;
    }
    robotApi_->turn(direction.c_str(), angleDeg);
    return true;
  }

  if (functionName == "setPose") {
    const String pose = args["pose"] | "";
    if (!(equalsIgnoreCase(pose, "sit") || equalsIgnoreCase(pose, "stand") || equalsIgnoreCase(pose, "idle"))) {
      messageOut = "invalid pose; expected sit/stand/idle";
      return false;
    }
    robotApi_->setPose(pose.c_str());
    return true;
  }

  if (functionName == "stop") {
    robotApi_->stop();
    return true;
  }

  if (functionName == "setGaitSpeed") {
    const float speed = args["speed"] | 1.0f;
    if (speed < 0.1f || speed > 2.0f) {
      messageOut = "invalid gait speed; expected 0.1-2.0";
      return false;
    }
    robotApi_->setGaitSpeed(speed);
    return true;
  }

  messageOut = "unknown function: " + functionName;
  return false;
}
