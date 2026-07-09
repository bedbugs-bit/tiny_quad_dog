#include "llm_control.h"

#define LLM_API_ENDPOINT "https://api.openai.com/v1/chat/completions"
#define LLM_API_KEY "REPLACE_WITH_YOUR_KEY"

namespace {
String buildSystemPrompt() {
  return String(
      "You are a quadruped robot command router. Respond with strict JSON only. "
      "Available functions: move(direction, speed, duration_ms), turn(direction, angle_deg), "
      "setPose(pose), stop(), setGaitSpeed(speed). "
      "Return {\"function\": \"move\", \"args\": {...}}. No prose.");
}

String sanitizeJson(const String& payload) {
  String out = payload;
  out.replace("\n", " ");
  out.trim();
  return out;
}
}  // namespace

LlmControl::LlmControl() {}

void LlmControl::begin(const char* ssid, const char* password, RobotApi* robotApi) {
  robotApi_ = robotApi;
  connected_ = connectWiFi(ssid, password);
}

void LlmControl::update() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleSerialCommand(line);
    }
  }
}

void LlmControl::handleSerialCommand(const String& command) {
  pendingCommand_ = command;
  String response;
  if (sendPrompt(command, response)) {
    parseAndDispatch(response);
  }
}

bool LlmControl::connectWiFi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool LlmControl::sendPrompt(const String& prompt, String& response) {
  if (!connected_) {
    return false;
  }

  if (!wifiClient_.connect("api.openai.com", 443)) {
    return false;
  }

  String body = String("{\"model\":\"gpt-4o-mini\",\"messages\":[{\"role\":\"system\",\"content\":\"") + buildSystemPrompt() + String("\"},{\"role\":\"user\",\"content\":\"") + prompt + String("\"}],\"temperature\":0} ");

  wifiClient_.println("POST /v1/chat/completions HTTP/1.1");
  wifiClient_.println("Host: api.openai.com");
  wifiClient_.println("Content-Type: application/json");
  wifiClient_.println(String("Authorization: Bearer ") + LLM_API_KEY);
  wifiClient_.print("Content-Length: ");
  wifiClient_.println(body.length());
  wifiClient_.println("Connection: close");
  wifiClient_.println();
  wifiClient_.print(body);

  unsigned long start = millis();
  while (wifiClient_.connected() && millis() - start < 5000) {
    if (wifiClient_.available()) {
      String line = wifiClient_.readStringUntil('\n');
      if (line.startsWith("{")) {
        response = sanitizeJson(line);
        break;
      }
    }
  }

  wifiClient_.stop();
  return response.length() > 0;
}

bool LlmControl::parseAndDispatch(const String& payload) {
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("LLM parse failed");
    return false;
  }

  const char* functionName = doc["function"]; 
  if (!functionName) {
    Serial.println("LLM missing function");
    return false;
  }

  JsonObject args = doc["args"].as<JsonObject>();
  if (String(functionName) == "move") {
    const char* direction = args["direction"] | "forward";
    const float speed = args["speed"] | 0.5f;
    const int duration = args["duration_ms"] | 1000;
    robotApi_->move(direction, speed, duration);
  } else if (String(functionName) == "turn") {
    const char* direction = args["direction"] | "left";
    const float angle = args["angle_deg"] | 30.0f;
    robotApi_->turn(direction, angle);
  } else if (String(functionName) == "setPose") {
    const char* pose = args["pose"] | "stand";
    robotApi_->setPose(pose);
  } else if (String(functionName) == "stop") {
    robotApi_->stop();
  } else if (String(functionName) == "setGaitSpeed") {
    const float speed = args["speed"] | 1.0f;
    robotApi_->setGaitSpeed(speed);
  }
  return true;
}
