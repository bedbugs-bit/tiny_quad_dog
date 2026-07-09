#include <Arduino.h>
#include <Wire.h>
#include "leg_kinematics.h"
#include "servo_driver.h"
#include "gait_engine.h"
#include "robot_api.h"
#include "llm_control.h"

LegKinematics kinematics;
ServoDriver servoDriver;
GaitEngine gaitEngine;
RobotApi robotApi;
LlmControl llmControl;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servoDriver.begin(0x40);
  gaitEngine.begin(&kinematics, &servoDriver);
  robotApi.begin(&gaitEngine);
  llmControl.begin("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD", &robotApi);

  gaitEngine.standIdle();
  Serial.println("Quadruped firmware initialized. Send a natural-language command or use Serial commands.");
}

void loop() {
  servoDriver.update();
  gaitEngine.update();
  llmControl.update();
}
