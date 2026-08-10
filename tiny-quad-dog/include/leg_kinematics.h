#pragma once

// Deliberately free of Arduino.h: this header only needs fixed-width ints and
// <cmath>, which keeps the kinematics math portable to a host/native build so
// it can be unit tested without any ESP32 hardware (see test/test_leg_kinematics.cpp).
#include <cstdint>

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

  // ESTIMATES, not measurements -- derived from the overall scale of
  // assembly.stl (chassis ~87x67x18mm, front/rear hip spacing ~127mm, the
  // single largest leg part in the mesh spans ~77mm), not from identifying
  // individual link parts in the CAD (the mesh has no part names/labels, so
  // upper-leg vs. lower-leg segments can't be told apart with confidence).
  // These replace the previous placeholder (35/45mm, max 80mm reach) with
  // numbers proportioned to the real hip spacing instead of an arbitrary
  // small value, but they still MUST be replaced with calipers-on-the-robot
  // measurements before flight -- see README.md's calibration notes.
  static constexpr float kLinkLengthA = 40.0f; // Upper leg (thigh), hip joint to knee.
  static constexpr float kLinkLengthB = 65.0f; // Lower leg (shin), knee joint to foot.
};
