#include "servo_driver.h"

namespace {
float clampFloat(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}
}  // namespace

ServoDriver::ServoDriver() : pwm_(0x40) {}

ServoDriver::~ServoDriver() {}

void ServoDriver::begin(uint8_t address) {
  pwm_ = Adafruit_PWMServoDriver(address);
  pwm_.begin();
  pwm_.setOscillatorFrequency(27000000);
  pwm_.setPWMFreq(50);
  delay(10);
}

void ServoDriver::setAngle(uint8_t channel, float angleDeg, const JointCalibration& calibration) {
  float finalAngle = angleDeg + calibration.offsetDeg;
  if (calibration.invert) {
    finalAngle = 180.0f - finalAngle;
  }
  finalAngle = clampFloat(finalAngle, calibration.minAngleDeg, calibration.maxAngleDeg);
  writeChannel(channel, finalAngle);
  channels_[channel].currentAngle = finalAngle;
  channels_[channel].targetAngle = finalAngle;
  channels_[channel].active = false;
}

void ServoDriver::setAngleRaw(uint8_t channel, float angleDeg) {
  float finalAngle = clampFloat(angleDeg, 0.0f, 180.0f);
  writeChannel(channel, finalAngle);
  channels_[channel].currentAngle = finalAngle;
  channels_[channel].targetAngle = finalAngle;
  channels_[channel].active = false;
}

void ServoDriver::interpolateTo(uint8_t channel, float targetAngleDeg, uint32_t durationMs, const JointCalibration& calibration) {
  float finalTarget = targetAngleDeg + calibration.offsetDeg;
  if (calibration.invert) {
    finalTarget = 180.0f - finalTarget;
  }
  finalTarget = clampFloat(finalTarget, calibration.minAngleDeg, calibration.maxAngleDeg);

  channels_[channel].targetAngle = finalTarget;
  channels_[channel].startMs = millis();
  channels_[channel].durationMs = durationMs;
  channels_[channel].active = durationMs > 0;
  if (!channels_[channel].active) {
    writeChannel(channel, finalTarget);
    channels_[channel].currentAngle = finalTarget;
  }
}

void ServoDriver::update() {
  for (uint8_t i = 0; i < 16; ++i) {
    ChannelState& state = channels_[i];
    if (!state.active) {
      continue;
    }

    const uint32_t now = millis();
    const uint32_t elapsed = now - state.startMs;
    if (elapsed >= state.durationMs) {
      state.currentAngle = state.targetAngle;
      state.active = false;
      writeChannel(i, state.currentAngle);
      continue;
    }

    const float progress = static_cast<float>(elapsed) / static_cast<float>(state.durationMs);
    const float current = state.currentAngle + (state.targetAngle - state.currentAngle) * progress;
    state.currentAngle = current;
    writeChannel(i, current);
  }
}

float ServoDriver::getCurrentAngle(uint8_t channel) const {
  return channels_[channel].currentAngle;
}

uint16_t ServoDriver::angleToPulse(float angleDeg) const {
  const int pulse = static_cast<int>(map(angleDeg, 0.0f, 180.0f, 102.0f, 512.0f));
  return static_cast<uint16_t>(pulse);
}

void ServoDriver::writeChannel(uint8_t channel, float angleDeg) {
  const uint16_t pulse = angleToPulse(angleDeg);
  pwm_.setPWM(channel, 0, pulse);
}
