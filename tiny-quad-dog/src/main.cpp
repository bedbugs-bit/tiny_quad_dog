#include <Arduino.h>
#include <Wire.h>
#include "leg_kinematics.h"
#include "servo_driver.h"
#include "gait_engine.h"
#include "robot_api.h"
#include "command_interface.h"

LegKinematics kinematics;
ServoDriver servoDriver;
GaitEngine gaitEngine;
RobotApi robotApi;
CommandInterface commandInterface;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servoDriver.begin(0x40);
  gaitEngine.begin(&kinematics, &servoDriver);
  robotApi.begin(&gaitEngine);
  commandInterface.begin(&robotApi);

#if TQD_ENABLE_WIFI_SERVER
  // Only attempted when built with -D TQD_ENABLE_WIFI_SERVER=1 (see
  // platformio.ini). Replace with real credentials before enabling; leave
  // disabled (the default) to control the robot purely over USB serial.
  commandInterface.beginWifi("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD");
#endif

  gaitEngine.standIdle();
  Serial.println("{\"event\":\"ready\",\"message\":\"tiny_quad_dog firmware initialized; send JSON commands over serial\"}");
}

void loop() {
  servoDriver.update();
  robotApi.update();
  gaitEngine.update();
  commandInterface.update();
}
