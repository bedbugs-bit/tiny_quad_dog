#include "robot_api.h"

namespace {
float clampFloat(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

bool equalsIgnoreCase(const char* lhs, const char* rhs) {
  return strcasecmp(lhs, rhs) == 0;
}
}  // namespace

RobotApi::RobotApi() {}

namespace {
unsigned long clampDurationMs(unsigned long v, unsigned long lo, unsigned long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

void RobotApi::begin(GaitEngine* gaitEngine) {
  gaitEngine_ = gaitEngine;
}

void RobotApi::move(const char* direction, float speed, int durationMs) {
  if (gaitEngine_ == nullptr) {
    return;
  }

  const float clampedSpeed = clampFloat(speed, 0.0f, 2.0f);
  const unsigned long clampedDuration = clampDurationMs(static_cast<unsigned long>(durationMs), 100, 10000);

  if (equalsIgnoreCase(direction, "forward")) {
    gaitEngine_->walkForward(clampedSpeed, 18.0f);
  } else if (equalsIgnoreCase(direction, "backward")) {
    gaitEngine_->walkBackward(clampedSpeed, 18.0f);
  } else if (equalsIgnoreCase(direction, "left")) {
    gaitEngine_->turnLeft(clampedSpeed, 18.0f);
  } else if (equalsIgnoreCase(direction, "right")) {
    gaitEngine_->turnRight(clampedSpeed, 18.0f);
  }
  // Schedule stop after duration
  commandEndMs_ = millis() + clampedDuration;
}

void RobotApi::turn(const char* direction, float angleDeg) {
  if (gaitEngine_ == nullptr) {
    return;
  }
  const float clampedAngle = clampFloat(angleDeg, -90.0f, 90.0f);
  if (equalsIgnoreCase(direction, "left")) {
    gaitEngine_->turnLeft(1.0f, 12.0f + fabsf(clampedAngle));
  } else if (equalsIgnoreCase(direction, "right")) {
    gaitEngine_->turnRight(1.0f, 12.0f + fabsf(clampedAngle));
  }
  // approximate duration proportional to angle magnitude (ms)
  const unsigned long dur = static_cast<unsigned long>(constrain(static_cast<int>(fabsf(clampedAngle) * 20.0f), 200, 5000));
  commandEndMs_ = millis() + dur;
}

void RobotApi::setPose(const char* poseName) {
  if (gaitEngine_ == nullptr) {
    return;
  }
  if (equalsIgnoreCase(poseName, "sit")) {
    gaitEngine_->sitDown();
  } else if (equalsIgnoreCase(poseName, "stand")) {
    gaitEngine_->standUp();
  } else if (equalsIgnoreCase(poseName, "idle")) {
    gaitEngine_->standIdle();
  }
}

void RobotApi::stop() {
  if (gaitEngine_ != nullptr) {
    gaitEngine_->standIdle();
  }
}

void RobotApi::setGaitSpeed(float speed) {
  if (gaitEngine_ == nullptr) {
    return;
  }
  gaitEngine_->setStepParameters(12.0f, 18.0f, static_cast<uint32_t>(500.0f / clampFloat(speed, 0.1f, 2.0f)));
}

void RobotApi::update() {
  if (gaitEngine_ == nullptr) return;
  if (commandEndMs_ == 0) return;
  const unsigned long now = millis();
  // handle rollover safety
  if ((long)(now - commandEndMs_) >= 0) {
    // time expired: settle into idle stance
    gaitEngine_->standIdle();
    commandEndMs_ = 0;
  }
}

String RobotApi::getStatusString() const {
  String s = "tiny_quad_dog";
  if (gaitEngine_ == nullptr) {
    s += ":gait=uninit";
  } else {
    s += ":gait=ok";
  }
  if (commandEndMs_ != 0) {
    s += ":timed_cmd=running";
  }
  return s;
}
