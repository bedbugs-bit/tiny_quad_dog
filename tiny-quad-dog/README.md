# Tiny Quad Dog

## Overview
This PlatformIO project provides a starter firmware skeleton for an ESP32-based quadruped robot with:
- PCA9685-based servo control over I2C
- Analytical leg kinematics with configurable link lengths and calibration offsets
- A non-blocking gait engine for idle, walking, turning, sitting, standing, and playful motion
- A simple command API that can be driven from serial or a future LLM layer

## Calibration notes
1. Measure the actual linkage lengths and update the constants in [src/leg_kinematics.cpp](src/leg_kinematics.cpp) for the two link segments.
2. Set the zero offsets and angle limits in the calibration struct initialization in [src/leg_kinematics.cpp](src/leg_kinematics.cpp) to match the physical servo orientation and mechanical travel.
3. Test each gait separately by uncommenting the relevant call in the setup routine or by sending serial commands once the firmware is running.

## Suggested test sequence
- Flash the firmware and confirm that the PCA9685 responds on I2C.
- Start with standIdle() so all servos settle into a neutral pose.
- Then test walkForward(), walkBackward(), turnLeft(), turnRight(), sitDown(), standUp(), and wagOrShake() one-by-one.
- Only after the mechanics are verified should you enable the LLM control path and replace the placeholder Wi-Fi credentials and API key.

## LLM note
The included LLM interface is intentionally lightweight and uses a strict JSON response format. Because ESP32 memory and TLS are limited, it is usually safer to use a small companion service for the actual API call and have the ESP32 only receive the parsed command.
