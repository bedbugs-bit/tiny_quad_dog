#pragma once

#include <Arduino.h>

struct JointCalibration {
  float offsetDeg = 0.0f;
  float minAngleDeg = 0.0f;
  float maxAngleDeg = 180.0f;
  bool invert = false;

  JointCalibration(float offset = 0.0f, float min = 0.0f, float max = 180.0f, bool invertFlag = false)
      : offsetDeg(offset), minAngleDeg(min), maxAngleDeg(max), invert(invertFlag) {}
};

struct LegCalibration {
  JointCalibration joint1;
  JointCalibration joint2;
  JointCalibration swing;
};

class LegKinematics {
public:
  LegKinematics();

  void setDefaultCalibrations();

  // Forward kinematics for a 2-link sagittal mechanism.
  // The model uses link lengths L1/L2 and angles measured from the local hip frame.
  void calculateFootFromServos(float theta1Deg, float theta2Deg, float& xMm, float& yMm) const;

  // Inverse kinematics using the law-of-cosines / two-link analytic solution.
  // Returns false when the target point is out of reach for the configured link lengths.
  bool solveServosForFoot(float xMm, float yMm, float& theta1Deg, float& theta2Deg) const;

  // Swing servo is treated as a simple linear mapping from desired fore/aft angle to the servo angle.
  float mapSwingToServoAngle(uint8_t legIndex, float swingDeg) const;

  // Apply joint calibration to a raw target angle before sending to the servo driver.
  float applyJointCalibration(uint8_t legIndex, uint8_t jointIndex, float rawAngleDeg) const;

  LegCalibration calibrations[4];

  static constexpr float kLinkLengthA = 35.0f; // Placeholder: tune against the real linkage.
  static constexpr float kLinkLengthB = 45.0f; // Placeholder: tune against the real linkage.
};
