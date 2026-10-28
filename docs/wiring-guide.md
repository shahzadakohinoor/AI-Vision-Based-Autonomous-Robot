# Wiring Guide

1. Connect the L298N motor outputs to the four DC motors.
2. Connect the ESP32-CAM to a regulated 5V supply with a common ground.
3. Connect the HC-SR04 trigger and echo pins as specified in the firmware.
4. Use a voltage divider on the HC-SR04 echo signal where required.
5. Keep servo and motor power within their rated limits.
6. Disconnect power before changing wiring.

Verify every pin against the actual firmware before powering on.
