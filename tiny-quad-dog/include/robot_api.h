#pragma once

#include <Arduino.h>
#include "gait_engine.h"

class RobotApi {
public:
  RobotApi();

  void begin(GaitEngine* gaitEngine);
  void move(const char* direction, float speed, int durationMs);
  void turn(const char* direction, float angleDeg);
  void setPose(const char* poseName);
  void stop();
  void setGaitSpeed(float speed);
  // Call from the main loop to handle timed commands (e.g. move duration)
  void update();

  // Returns a short status string for telemetry/debug
  String getStatusString() const;

private:
  GaitEngine* gaitEngine_ = nullptr;
  // If non-zero, the time (millis) at which a previously-issued timed
  // command should be considered finished and the robot returned to idle.
  unsigned long commandEndMs_ = 0;
};
