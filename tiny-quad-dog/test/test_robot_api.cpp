// Tests for RobotApi timed commands (host/native)
// Run with: pio test -e native
#include <unity.h>

#include "robot_api.h"
#include "gait_engine.h"
#include "leg_kinematics.h"

// Minimal fake ServoDriver that avoids hardware access during host tests.
class FakeServoDriver : public ServoDriver {
 public:
  FakeServoDriver() = default;
  void begin(uint8_t) override {}
  void setAngle(uint8_t, float, const JointCalibration&) override {}
  void setAngleRaw(uint8_t, float) override {}
  void interpolateTo(uint8_t, float, uint32_t, const JointCalibration&) override {}
  void update() override {}
  float getCurrentAngle(uint8_t) const override { return 90.0f; }
};

void setUp(void) {}
void tearDown(void) {}

void test_move_duration_stops() {
  LegKinematics kin;
  FakeServoDriver servo;
  GaitEngine gait;
  RobotApi api;

  gait.begin(&kin, &servo);
  api.begin(&gait);

  // Issue a short move
  api.move("forward", 1.0f, 200);
  String s1 = api.getStatusString();
  TEST_ASSERT_TRUE_MESSAGE(s1.indexOf("timed_cmd=running") >= 0, "timed command should be running after move()");

  // Wait longer than the duration and call update
  delay(300);
  api.update();
  String s2 = api.getStatusString();
  TEST_ASSERT_TRUE_MESSAGE(s2.indexOf("timed_cmd=running") < 0, "timed command should have finished after duration");
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_move_duration_stops);
  return UNITY_END();
}
