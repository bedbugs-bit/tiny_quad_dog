#include "gait_engine.h"

namespace {
float clampFloat(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}
}  // namespace

GaitEngine::GaitEngine() {
  for (auto& pose : legPoses_) {
    pose = LegPose(0.0f, 0.0f, 0.0f);
  }
  for (auto& angle : lastSentAngles_) {
    angle = 90.0f;
  }
}

void GaitEngine::begin(LegKinematics* kinematics, ServoDriver* servoDriver) {
  kinematics_ = kinematics;
  servoDriver_ = servoDriver;
  mode_ = Mode::Idle;
  modeStartMs_ = millis();
  standIdle();
}

void GaitEngine::setStepParameters(float stepHeightMm, float stepLengthMm, uint32_t cycleDurationMs) {
  stepHeightMm_ = stepHeightMm;
  stepLengthMm_ = stepLengthMm;
  cycleDurationMs_ = cycleDurationMs;
}

void GaitEngine::standIdle() {
  mode_ = Mode::Idle;
  modeStartMs_ = millis();
  for (uint8_t i = 0; i < 4; ++i) {
    legPoses_[i] = LegPose(0.0f, 0.0f, 0.0f);
  }
  applyToServos();
}

void GaitEngine::walkForward(float speed, float stepLengthMm) {
  walkSpeed_ = clampFloat(speed, 0.0f, 2.0f);
  stepLengthMm_ = stepLengthMm;
  mode_ = Mode::WalkForward;
  modeStartMs_ = millis();
}

void GaitEngine::walkBackward(float speed, float stepLengthMm) {
  walkSpeed_ = clampFloat(speed, 0.0f, 2.0f);
  stepLengthMm_ = stepLengthMm;
  mode_ = Mode::WalkBackward;
  modeStartMs_ = millis();
}

void GaitEngine::turnLeft(float speed, float stepLengthMm) {
  turnSpeed_ = clampFloat(speed, 0.0f, 2.0f);
  stepLengthMm_ = stepLengthMm;
  mode_ = Mode::TurnLeft;
  modeStartMs_ = millis();
}

void GaitEngine::turnRight(float speed, float stepLengthMm) {
  turnSpeed_ = clampFloat(speed, 0.0f, 2.0f);
  stepLengthMm_ = stepLengthMm;
  mode_ = Mode::TurnRight;
  modeStartMs_ = millis();
}

void GaitEngine::sitDown() {
  mode_ = Mode::SitDown;
  modeStartMs_ = millis();
}

void GaitEngine::standUp() {
  mode_ = Mode::StandUp;
  modeStartMs_ = millis();
}

void GaitEngine::wagOrShake() {
  mode_ = Mode::WagOrShake;
  modeStartMs_ = millis();
}

void GaitEngine::update() {
  switch (mode_) {
    case Mode::WalkForward:
      updateWalk(1.0f);
      break;
    case Mode::WalkBackward:
      updateWalk(-1.0f);
      break;
    case Mode::TurnLeft:
      updateTurn(1.0f);
      break;
    case Mode::TurnRight:
      updateTurn(-1.0f);
      break;
    case Mode::SitDown:
      updateSitStand(10.0f);
      break;
    case Mode::StandUp:
      updateSitStand(28.0f);
      break;
    case Mode::WagOrShake:
      updateWag();
      break;
    case Mode::Idle:
    default:
      break;
  }
  applyToServos();
}

const LegPose* GaitEngine::getLegPoses() const {
  return legPoses_.data();
}

void GaitEngine::applyToServos() {
  if (kinematics_ == nullptr || servoDriver_ == nullptr) {
    return;
  }

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const LegPose& pose = legPoses_[legIndex];
    float theta1Deg = 90.0f;
    float theta2Deg = 90.0f;
    if (kinematics_->solveServosForFoot(pose.x, pose.y, theta1Deg, theta2Deg)) {
      const float servo1 = kinematics_->applyJointCalibration(legIndex, 0, theta1Deg);
      const float servo2 = kinematics_->applyJointCalibration(legIndex, 1, theta2Deg);
      const float swing = kinematics_->mapSwingToServoAngle(legIndex, pose.swing);

      const uint8_t ch1 = static_cast<uint8_t>(legIndex * 3);
      const uint8_t ch2 = static_cast<uint8_t>(legIndex * 3 + 1);
      const uint8_t ch3 = static_cast<uint8_t>(legIndex * 3 + 2);
      servoDriver_->setAngleRaw(ch1, servo1);
      servoDriver_->setAngleRaw(ch2, servo2);
      servoDriver_->setAngleRaw(ch3, swing);
    }
  }
}

void GaitEngine::setAllLegs(const LegPose& pose) {
  for (auto& legPose : legPoses_) {
    legPose = pose;
  }
}

void GaitEngine::updateWalk(float directionSign) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float phase = static_cast<float>((elapsed % cycleDurationMs_) / static_cast<float>(cycleDurationMs_));
  const bool leftSwing = phase < 0.5f;

  const float rise = sinf(phase * 2.0f * 3.14159265f) * stepHeightMm_;
  const float stride = directionSign * stepLengthMm_ * walkSpeed_ * (phase < 0.5f ? 1.0f : -1.0f);

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const bool isSwing = (legIndex % 2 == 0) ? leftSwing : !leftSwing;
    LegPose pose(0.0f, 0.0f, 0.0f);
    pose.x = isSwing ? stride : 0.0f;
    pose.y = isSwing ? rise : 0.0f;
    pose.swing = isSwing ? 5.0f : -5.0f;
    legPoses_[legIndex] = pose;
  }
}

void GaitEngine::updateTurn(float turnSign) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float phase = static_cast<float>((elapsed % cycleDurationMs_) / static_cast<float>(cycleDurationMs_));
  const bool leftSwing = phase < 0.5f;
  const float rise = sinf(phase * 2.0f * 3.14159265f) * stepHeightMm_;
  const float stride = turnSign * stepLengthMm_ * turnSpeed_ * (phase < 0.5f ? 1.0f : -1.0f);

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const bool isSwing = (legIndex % 2 == 0) ? leftSwing : !leftSwing;
    LegPose pose(0.0f, 0.0f, 0.0f);
    pose.x = isSwing ? stride : 0.0f;
    pose.y = isSwing ? rise : 0.0f;
    pose.swing = isSwing ? 4.0f : -4.0f;
    legPoses_[legIndex] = pose;
  }
}

void GaitEngine::updateSitStand(float targetHeightMm) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float progress = clampFloat(static_cast<float>(elapsed) / 1500.0f, 0.0f, 1.0f);
  const float height = lerp(bodyHeightMm_, targetHeightMm, progress);
  bodyHeightMm_ = height;
  for (auto& pose : legPoses_) {
    pose.y = height;
    pose.swing = 0.0f;
  }
}

void GaitEngine::updateWag() {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float phase = static_cast<float>(elapsed % 1200) / 1200.0f;
  const float swing = sinf(phase * 2.0f * 3.14159265f) * 10.0f;
  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    legPoses_[legIndex] = LegPose(0.0f, 0.0f, swing);
  }
}
