#include "leg_kinematics.h"

#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;

float clampFloat(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}
}  // namespace

LegKinematics::LegKinematics() {
  setDefaultCalibrations();
}

void LegKinematics::setDefaultCalibrations() {
  for (uint8_t i = 0; i < 4; ++i) {
    calibrations[i].joint1 = JointCalibration(0.0f, 20.0f, 160.0f, false);
    calibrations[i].joint2 = JointCalibration(0.0f, 20.0f, 160.0f, false);
    calibrations[i].swing = JointCalibration(90.0f, 30.0f, 150.0f, false);
  }

  // Mirrored legs can be inverted here once the final mechanical orientation is confirmed.
  calibrations[1].joint1.invert = true;
  calibrations[1].joint2.invert = true;
  calibrations[1].swing.invert = true;
  calibrations[3].joint1.invert = true;
  calibrations[3].joint2.invert = true;
  calibrations[3].swing.invert = true;
}

void LegKinematics::calculateFootFromServos(float theta1Deg, float theta2Deg, float& xMm, float& yMm) const {
  const float theta1 = theta1Deg * kPi / 180.0f;
  const float theta2 = theta2Deg * kPi / 180.0f;
  xMm = kLinkLengthA * cosf(theta1) + kLinkLengthB * cosf(theta1 + theta2);
  yMm = kLinkLengthA * sinf(theta1) + kLinkLengthB * sinf(theta1 + theta2);
}

bool LegKinematics::solveServosForFoot(float xMm, float yMm, float& theta1Deg, float& theta2Deg) const {
  const float r = sqrtf(xMm * xMm + yMm * yMm);
  if (r > (kLinkLengthA + kLinkLengthB) + 1.0f || r < fabsf(kLinkLengthA - kLinkLengthB) - 1.0f) {
    return false;
  }

  // Law of cosines for the second joint. With theta2 defined (per
  // calculateFootFromServos) as the bend away from a fully-extended leg,
  // r^2 = L1^2 + L2^2 + 2*L1*L2*cos(theta2) -- note the *positive* cross
  // term, since r is maximal (L1+L2) when theta2 = 0 (leg straight).
  const float cosTheta2 = clampFloat((r * r - kLinkLengthA * kLinkLengthA - kLinkLengthB * kLinkLengthB) / (2.0f * kLinkLengthA * kLinkLengthB), -1.0f, 1.0f);
  const float theta2 = acosf(cosTheta2);

  // First joint uses the geometry of the two-link chain.
  const float alpha = atan2f(yMm, xMm);
  const float beta = atan2f(kLinkLengthB * sinf(theta2), kLinkLengthA + kLinkLengthB * cosf(theta2));
  theta1Deg = (alpha - beta) * 180.0f / kPi;
  theta2Deg = theta2 * 180.0f / kPi;

  return true;
}

float LegKinematics::mapSwingToServoAngle(uint8_t legIndex, float swingDeg) const {
  if (legIndex >= 4) {
    return 90.0f;
  }
  float mapped = swingDeg + calibrations[legIndex].swing.offsetDeg;
  if (calibrations[legIndex].swing.invert) {
    mapped = 180.0f - mapped;
  }
  return clampFloat(mapped, calibrations[legIndex].swing.minAngleDeg, calibrations[legIndex].swing.maxAngleDeg);
}

float LegKinematics::applyJointCalibration(uint8_t legIndex, uint8_t jointIndex, float rawAngleDeg) const {
  if (legIndex >= 4) {
    return clampFloat(rawAngleDeg, 0.0f, 180.0f);
  }

  const JointCalibration* calibration = nullptr;
  if (jointIndex == 0) {
    calibration = &calibrations[legIndex].joint1;
  } else if (jointIndex == 1) {
    calibration = &calibrations[legIndex].joint2;
  } else if (jointIndex == 2) {
    calibration = &calibrations[legIndex].swing;
  }
  if (calibration == nullptr) {
    return clampFloat(rawAngleDeg, 0.0f, 180.0f);
  }

  float angle = rawAngleDeg + calibration->offsetDeg;
  if (calibration->invert) {
    angle = 180.0f - angle;
  }
  return clampFloat(angle, calibration->minAngleDeg, calibration->maxAngleDeg);
}
