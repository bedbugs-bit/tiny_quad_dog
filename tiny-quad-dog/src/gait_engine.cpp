#include "gait_engine.h"

namespace {
constexpr float kPi = 3.14159265f;

// true for the {front-right, rear-left} diagonal pair (legs 1, 2), false for
// the {front-left, rear-right} pair (legs 0, 3). See the leg index
// convention comment in gait_engine.h.
constexpr bool kDiagonalGroupB[4] = {false, true, true, false};

// Sign of a leg's stride during in-place turning: left-side legs and
// right-side legs must stride in opposite directions to rotate the body.
constexpr float kTurnDirection[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

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
  // Clamp to conservative safe ranges to avoid over-stretching legs.
  stepHeightMm_ = clampFloat(stepHeightMm, 0.0f, 40.0f);
  stepLengthMm_ = clampFloat(stepLengthMm, 0.0f, 50.0f);
  cycleDurationMs_ = constrain(cycleDurationMs, 200u, 2000u);
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
  heightAtModeStart_ = bodyHeightMm_;
}

void GaitEngine::standUp() {
  mode_ = Mode::StandUp;
  modeStartMs_ = millis();
  heightAtModeStart_ = bodyHeightMm_;
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
      updateSitStand(20.0f);
      break;
    case Mode::StandUp:
      updateSitStand(55.0f);
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

LegPose GaitEngine::computeStepPose(float legPhase01, float strideAmplitudeMm) const {
  LegPose pose;
  if (legPhase01 < 0.5f) {
    // Swing sub-phase: foot is off the ground, sweeping forward to plant
    // ahead of the body. Lift peaks at the midpoint of the swing.
    const float s = legPhase01 / 0.5f;
    pose.x = lerp(-strideAmplitudeMm, strideAmplitudeMm, s);
    pose.y = bodyHeightMm_ - sinf(s * kPi) * stepHeightMm_;
  } else {
    // Stance sub-phase: foot is planted at full standing extension and
    // sweeps backward relative to the body -- this is the stroke that
    // actually pushes/rotates the body, since this foot has ground contact.
    const float s = (legPhase01 - 0.5f) / 0.5f;
    pose.x = lerp(strideAmplitudeMm, -strideAmplitudeMm, s);
    pose.y = bodyHeightMm_;
  }
  // Small continuous hip-yaw sway to help shift weight toward the
  // supporting diagonal; purely cosmetic/stability-assist and safe to
  // tune down to 0 once the real robot's balance is characterized.
  pose.swing = 5.0f * cosf(legPhase01 * 2.0f * kPi);
  return pose;
}

void GaitEngine::updateWalk(float directionSign) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float globalPhase = static_cast<float>(elapsed % cycleDurationMs_) / static_cast<float>(cycleDurationMs_);
  const float strideAmplitude = 0.5f * stepLengthMm_ * walkSpeed_ * directionSign;

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const float phaseOffset = kDiagonalGroupB[legIndex] ? 0.5f : 0.0f;
    const float legPhase = fmodf(globalPhase + phaseOffset, 1.0f);
    legPoses_[legIndex] = computeStepPose(legPhase, strideAmplitude);
  }
}

void GaitEngine::updateTurn(float turnSign) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float globalPhase = static_cast<float>(elapsed % cycleDurationMs_) / static_cast<float>(cycleDurationMs_);

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const float phaseOffset = kDiagonalGroupB[legIndex] ? 0.5f : 0.0f;
    const float legPhase = fmodf(globalPhase + phaseOffset, 1.0f);
    const float strideAmplitude = 0.5f * stepLengthMm_ * turnSpeed_ * turnSign * kTurnDirection[legIndex];
    legPoses_[legIndex] = computeStepPose(legPhase, strideAmplitude);
  }
}

void GaitEngine::updateSitStand(float targetHeightMm) {
  const uint32_t elapsed = millis() - modeStartMs_;
  const float progress = clampFloat(static_cast<float>(elapsed) / 1500.0f, 0.0f, 1.0f);
  const float height = lerp(heightAtModeStart_, targetHeightMm, progress);
  bodyHeightMm_ = height;
  for (auto& pose : legPoses_) {
    pose.x = 0.0f;
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
