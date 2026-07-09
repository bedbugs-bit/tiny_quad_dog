#pragma once

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "robot_api.h"

class LlmControl {
public:
  LlmControl();
  void begin(const char* ssid, const char* password, RobotApi* robotApi);
  void update();
  void handleSerialCommand(const String& command);

private:
  RobotApi* robotApi_ = nullptr;
  WiFiClientSecure wifiClient_;
  bool connected_ = false;
  String pendingCommand_;

  bool connectWiFi(const char* ssid, const char* password);
  bool sendPrompt(const String& prompt, String& response);
  bool parseAndDispatch(const String& payload);
};
