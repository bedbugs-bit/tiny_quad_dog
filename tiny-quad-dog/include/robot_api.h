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

private:
  GaitEngine* gaitEngine_ = nullptr;
};
