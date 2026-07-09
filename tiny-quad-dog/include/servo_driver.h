#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg_kinematics.h"

class ServoDriver {
public:
  struct ChannelState {
    bool active = false;
    float currentAngle = 90.0f;
    float targetAngle = 90.0f;
    uint32_t startMs = 0;
    uint32_t durationMs = 0;
  };

  ServoDriver();

  void begin(uint8_t address = 0x40);
  void setAngle(uint8_t channel, float angleDeg, const JointCalibration& calibration);
  void setAngleRaw(uint8_t channel, float angleDeg);
  void interpolateTo(uint8_t channel, float targetAngleDeg, uint32_t durationMs, const JointCalibration& calibration);
  void update();
  float getCurrentAngle(uint8_t channel) const;

private:
  Adafruit_PWMServoDriver pwm_;
  ChannelState channels_[16];
  uint16_t angleToPulse(float angleDeg) const;
  void writeChannel(uint8_t channel, float angleDeg);
};
