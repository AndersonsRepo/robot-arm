# Archived: ESP32 Operator Tracker

This directory contains the legacy ESP32 firmware for IMU-based operator tracking.

**Status**: Archived - replaced by ESP32 arm driver firmware that receives joint commands from Raspberry Pi.

The operator tracker firmware:
- Read 3× MPU-9250 IMU sensors
- Performed orientation fusion
- Transmitted operator pose via WiFi UDP or Bluetooth Serial

See `firmware/esp32/arm_driver/` for the new motor driver firmware.

