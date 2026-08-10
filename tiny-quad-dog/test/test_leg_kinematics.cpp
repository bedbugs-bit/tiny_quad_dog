// Host-side (no hardware) tests for the pure-math leg IK/FK.
// Run with:  pio test -e native
#include <unity.h>

#include <cmath>

#include "leg_kinematics.h"

namespace {
constexpr float kEps = 0.05f;
}

void setUp(void) {}
void tearDown(void) {}

// Forward kinematics followed by inverse kinematics should recover a servo
// pair that produces (very nearly) the same foot position. This is the
// property that matters for the gait engine: it only ever calls
// solveServosForFoot() on positions it derived from calculateFootFromServos(),
// so a broken round trip here means every gait would misplace its feet.
void test_forward_inverse_roundtrip() {
  LegKinematics kin;

  for (float theta1 = 20.0f; theta1 <= 160.0f; theta1 += 10.0f) {
    for (float theta2 = 20.0f; theta2 <= 160.0f; theta2 += 10.0f) {
      float x = 0.0f, y = 0.0f;
      kin.calculateFootFromServos(theta1, theta2, x, y);

      float recoveredTheta1 = 0.0f, recoveredTheta2 = 0.0f;
      const bool reachable = kin.solveServosForFoot(x, y, recoveredTheta1, recoveredTheta2);
      TEST_ASSERT_TRUE_MESSAGE(reachable, "point derived from FK must be reachable by IK");

      float x2 = 0.0f, y2 = 0.0f;
      kin.calculateFootFromServos(recoveredTheta1, recoveredTheta2, x2, y2);

      TEST_ASSERT_FLOAT_WITHIN(kEps, x, x2);
      TEST_ASSERT_FLOAT_WITHIN(kEps, y, y2);
    }
  }
}

// Points further than L1+L2 or closer than |L1-L2| from the hip are outside
// the mechanical reach of the two-link leg and must be rejected rather than
// silently clamped -- callers rely on the bool return to skip a bad pose.
void test_reachability_limits() {
  LegKinematics kin;
  const float maxReach = LegKinematics::kLinkLengthA + LegKinematics::kLinkLengthB;
  const float minReach = fabsf(LegKinematics::kLinkLengthA - LegKinematics::kLinkLengthB);

  float theta1 = 0.0f, theta2 = 0.0f;
  TEST_ASSERT_FALSE(kin.solveServosForFoot(maxReach + 20.0f, 0.0f, theta1, theta2));
  TEST_ASSERT_TRUE(kin.solveServosForFoot(maxReach - 5.0f, 0.0f, theta1, theta2));
  if (minReach > 5.0f) {
    TEST_ASSERT_FALSE(kin.solveServosForFoot(minReach - 5.0f, 0.0f, theta1, theta2));
  }
}

// applyJointCalibration must add the offset, mirror inverted joints around
// 180 - angle, and clamp to the configured mechanical travel limits.
void test_calibration_offset_and_invert() {
  LegKinematics kin;
  kin.calibrations[0].joint1 = JointCalibration(/*offset=*/5.0f, /*min=*/0.0f, /*max=*/180.0f, /*invert=*/false);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 95.0f, kin.applyJointCalibration(0, 0, 90.0f));

  kin.calibrations[0].joint2 = JointCalibration(/*offset=*/0.0f, /*min=*/0.0f, /*max=*/180.0f, /*invert=*/true);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 110.0f, kin.applyJointCalibration(0, 1, 70.0f));  // 180 - 70

  kin.calibrations[0].joint1 = JointCalibration(/*offset=*/0.0f, /*min=*/30.0f, /*max=*/150.0f, /*invert=*/false);
  TEST_ASSERT_FLOAT_WITHIN(kEps, 150.0f, kin.applyJointCalibration(0, 0, 179.0f));
  TEST_ASSERT_FLOAT_WITHIN(kEps, 30.0f, kin.applyJointCalibration(0, 0, -50.0f));
}

// Mirrored legs (index 1 and 3 by the default calibration) must invert the
// swing angle so a positive "swing forward" command moves both sides of the
// body the same physical direction rather than mirrored ones.
void test_swing_mapping_mirrors_opposite_legs() {
  LegKinematics kin;
  const float leftSwing = kin.mapSwingToServoAngle(0, 20.0f);
  const float rightSwing = kin.mapSwingToServoAngle(1, 20.0f);
  // Leg 1 is configured as inverted, so the same logical swing command must
  // land on the opposite side of 90 degrees relative to leg 0.
  TEST_ASSERT_TRUE((leftSwing - 90.0f) * (rightSwing - 90.0f) <= 0.0f);
}

void test_swing_mapping_clamps_to_limits() {
  LegKinematics kin;
  const float clamped = kin.mapSwingToServoAngle(0, 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(kEps, kin.calibrations[0].swing.maxAngleDeg, clamped);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_forward_inverse_roundtrip);
  RUN_TEST(test_reachability_limits);
  RUN_TEST(test_calibration_offset_and_invert);
  RUN_TEST(test_swing_mapping_mirrors_opposite_legs);
  RUN_TEST(test_swing_mapping_clamps_to_limits);
  return UNITY_END();
}
