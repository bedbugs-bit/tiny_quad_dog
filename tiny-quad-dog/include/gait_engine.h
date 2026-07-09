#pragma once

#include <Arduino.h>
#include <array>
#include "leg_kinematics.h"
#include "servo_driver.h"

struct LegPose {
  float x = 0.0f;
  float y = 0.0f;
  float swing = 0.0f;

  LegPose(float xMm = 0.0f, float yMm = 0.0f, float swingDeg = 0.0f)
      : x(xMm), y(yMm), swing(swingDeg) {}
};

class GaitEngine {
public:
  GaitEngine();

  void begin(LegKinematics* kinematics, ServoDriver* servoDriver);
  void setStepParameters(float stepHeightMm, float stepLengthMm, uint32_t cycleDurationMs);

  void standIdle();
  void walkForward(float speed, float stepLengthMm);
  void walkBackward(float speed, float stepLengthMm);
  void turnLeft(float speed, float stepLengthMm);
  void turnRight(float speed, float stepLengthMm);
  void sitDown();
  void standUp();
  void wagOrShake();

  void update();

  const LegPose* getLegPoses() const;

private:
  enum class Mode {
    Idle,
    WalkForward,
    WalkBackward,
    TurnLeft,
    TurnRight,
    SitDown,
    StandUp,
    WagOrShake
  };

  LegKinematics* kinematics_ = nullptr;
  ServoDriver* servoDriver_ = nullptr;
  Mode mode_ = Mode::Idle;
  std::array<LegPose, 4> legPoses_{};
  std::array<float, 12> lastSentAngles_{};
  float stepHeightMm_ = 12.0f;
  float stepLengthMm_ = 18.0f;
  uint32_t cycleDurationMs_ = 500;
  uint32_t modeStartMs_ = 0;
  float bodyHeightMm_ = 28.0f;
  float walkSpeed_ = 1.0f;
  float turnSpeed_ = 1.0f;

  void applyToServos();
  void setAllLegs(const LegPose& pose);
  void updateWalk(float directionSign);
  void updateTurn(float turnSign);
  void updateSitStand(float targetHeightMm);
  void updateWag();
};
