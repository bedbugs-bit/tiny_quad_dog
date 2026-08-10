#pragma once

#include <Arduino.h>
#include <array>
#include "leg_kinematics.h"
#include "servo_driver.h"

// Leg index convention used throughout GaitEngine and LegKinematics:
//   0 = front-left   1 = front-right
//   2 = rear-left    3 = rear-right
// This matches LegKinematics::setDefaultCalibrations(), which mirrors legs
// 1 and 3 (the right side). Diagonal trot pairs are {0,3} and {1,2}: each
// pair swings together while the other pair stays planted. If the physical
// wiring doesn't match this layout, swap the affected servo channel bases
// (legIndex * 3) in main.cpp/wiring rather than the gait math.
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
  // ~half of the estimated max leg reach (kLinkLengthA + kLinkLengthB =
  // 105mm) -- a comfortable knee bend for standing. Re-tune once the real
  // link lengths are measured; see the estimate note in leg_kinematics.h.
  float bodyHeightMm_ = 55.0f;
  float heightAtModeStart_ = 55.0f;
  float walkSpeed_ = 1.0f;
  float turnSpeed_ = 1.0f;

  void applyToServos();
  void setAllLegs(const LegPose& pose);
  void updateWalk(float directionSign);
  void updateTurn(float turnSign);
  void updateSitStand(float targetHeightMm);
  void updateWag();

  // Shared trot step: first half of legPhase01 is the swing sub-phase
  // (foot lifted off the ground and moved from -strideAmplitudeMm to
  // +strideAmplitudeMm to get ready for the next stance), second half is
  // the stance sub-phase (foot planted at bodyHeightMm_ and swept from
  // +strideAmplitudeMm back to -strideAmplitudeMm, which is what actually
  // drives/rotates the body since that foot is in contact with the ground).
  LegPose computeStepPose(float legPhase01, float strideAmplitudeMm) const;
};
