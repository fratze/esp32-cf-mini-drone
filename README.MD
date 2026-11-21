# CHERGLOW CF-Drone

Flight controller firmware for ESP32 mini drones. You can fly with the web remote on a phone, a radio receiver using CRSF (ELRS) or SBUS, or a ground station using MAVLink. The firmware comes from the [CHERGLOW CF-Drone open-source project](https://oshwhub.com/songge8/project_qqqyfdkm).

## Build the firmware

1. Open `esp32-cf-mini-drone.ino` in Arduino IDE 2.3.x or later.
2. In Boards Manager, install **esp32 by Espressif Systems** version **3.3.1**. In Library Manager, install **MAVLink** version **2.0.35**.
3. Select the board that matches your chip and click **Verify** to compile.

With the same board package and library installed, you can also compile from the repository directory with Arduino CLI. For a standard ESP32, run:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

For an ESP32-S3, use `esp32:esp32:esp32s3`. For an ESP32-C3, use `esp32:esp32:esp32c3`. The C3 configuration disables Wi-Fi and the web remote by default.

## Hardware

The firmware supports MPU6500 and MPU9250 motion sensors. Board-specific pins are listed in `board_config.h`.

| CHERGLOW hardware | Details | Links |
| --- | --- | --- |
| E1_MINI | Open-source mini drone. The official expansion module is designed for altitude hold and hovering. This firmware currently falls back to stabilize mode when the web remote requests altitude hold. | [Hardware files](https://oshwhub.com/songge8/project_qqqyfdkm) · [Kit](https://item.taobao.com/item.htm?ft=t&id=1098812979567) |
| E1_Nano | Extra-small drone in the E1 series. | [Hardware files](https://oshwhub.com/songge8/project_fnvbukbz) · [Kit](https://item.taobao.com/item.htm?ft=t&id=1099763930032) |
| RC1 | Open-source radio controller. The original project directs RC1 users to its official firmware while flight controller support is being updated. | [Hardware files](https://oshwhub.com/songge8/project_wajhbuzm) |

## Change log

| Date | Changes |
| --- | --- |
| 2026-07-18 | Improved how settings are saved. |
| 2026-06-24 | Reorganized settings by category. |
| 2026-06-18 | Improved messages on the web remote. |
| 2026-06-17 | Improved handling of motion sensor (IMU) faults. |
| 2026-05-21 | Improved attitude control. |
| 2026-05-20 | Adjusted the motor PWM frequency to reduce stress on the MOSFETs. |
| 2026-05-15 | Improved battery voltage readings and adjusted the low-battery thresholds. |
| 2026-04-30 | Updated low-battery protection to block arming, disarm at idle or land automatically, depending on flight state and voltage. Added minimum and maximum thrust settings (`MOT_THR_MIN` and `MOT_THR_MAX`) and updated the web remote. |
| 2026-04-28 | Added ESP32-S3 support. |
| 2026-04-17 | Fixed cut-off debug logs and improved the command help. |
| 2026-04-16 | Reduced routine remote-control logging. Removed the server's inaccurate network quality estimate. The web remote measures link quality in the browser. |
| 2026-04-15 | Added status LED patterns. A slow blink means armed, and a fast blink signals a warning such as low battery or a lost connection. Made motor pins and Wi-Fi credentials configurable from the console. Fixed the latency calculation and raised the web remote update rate to 20 Hz. The project recommends staying at or below 20 Hz for performance. Also updated general features. |
| 2026-03-28 | Reworked the web remote and removed redundant server code. |
| 2026-03-27 | Improved the web remote and added debugging tools. Added protection that stops the motors if the drone stays upside down too long. |
