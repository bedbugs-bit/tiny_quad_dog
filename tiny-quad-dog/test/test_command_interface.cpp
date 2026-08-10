// Tests for CommandInterface parsing and validation (host/native)
// Run with: pio test -e native
#include <unity.h>

#include "command_interface.h"
#include "robot_api.h"
#include "gait_engine.h"
#include "leg_kinematics.h"

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

void test_process_command_validation() {
  LegKinematics kin;
  FakeServoDriver servo;
  GaitEngine gait;
  RobotApi api;
  CommandInterface iface;

  gait.begin(&kin, &servo);
  api.begin(&gait);
  iface.begin(&api);

  // invalid move (bad speed)
  String bad = "{\"function\":\"move\", \"args\": {\"direction\": \"forward\", \"speed\": -1.0, \"duration_ms\": 500}}";
  String resp = iface.processCommand(bad);
  DynamicJsonDocument doc(256);
  deserializeJson(doc, resp);
  TEST_ASSERT_EQUAL_STRING("error", doc["status"] | "");

  // valid move
  String ok = "{\"function\":\"move\", \"args\": {\"direction\": \"forward\", \"speed\": 0.5, \"duration_ms\": 200}}";
  resp = iface.processCommand(ok);
  deserializeJson(doc, resp);
  TEST_ASSERT_EQUAL_STRING("ok", doc["status"] | "");

  // status returns structured info
  String st = "{\"function\":\"status\"}";
  resp = iface.processCommand(st);
  deserializeJson(doc, resp);
  TEST_ASSERT_EQUAL_STRING("ok", doc["status"] | "");
  TEST_ASSERT_TRUE(doc.containsKey("info"));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_process_command_validation);
  return UNITY_END();
}
