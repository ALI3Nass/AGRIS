# AGRIS

**A laser targeting system that uses computer vision to detect shapes and drive a pan/tilt servo gimbal to aim a laser at detected targets, mounted on a remotely driven mecanum platform.

## System overview

Two independent subsystems share the same metal frame:

**Vision/targeting** — a fixed OV2640 camera streams MJPEG video to a Python desktop app. The app detects target shapes using OpenCV, calculates the angular error from frame center, and sends absolute servo positions over UDP to the AI-Thinker ESP32. The ESP32 drives two MG996R servos and a laser to point at the target.

**Drive** — an ESP32-S1 connects to a DualShock 4 controller over Bluetooth and drives four mecanum wheels independently, letting the platform strafe and rotate in place.

```
OV2640 camera (fixed to frame)
  → MJPEG stream over Wi-Fi
  → agris.py on PC (shape detection, FOV-aware proportional control)
  → UDP servo commands  →  AI-Thinker ESP32
                             ├── MG996R pan servo
                             ├── MG996R tilt servo
                             └── laser module

DualShock 4 (Bluetooth)
  → ESP32-S1
      ├── mecanum wheel FL
      ├── mecanum wheel FR
      ├── mecanum wheel RL
      └── mecanum wheel RR
```

## Hardware

| Part | Details |
|---|---|
| Camera + servo controller | AI-Thinker ESP32-CAM (OV2640) |
| Drive controller | ESP32-S1 |
| Camera | OV2640, QVGA (320×240) for low latency |
| Servos | MG996R × 2 (pan/tilt) |
| Laser | LaserTree LT-40W-F23 (~5W optical, 12V/1.8A, PWM via signal wire) |
| Controller | DualShock 4 over Bluetooth (Bluepad32) |
| Host | Linux PC running agris.py |
| Frame | Laser-cut metal — DXF files in `CAD_Design/` |
| UDP port | 4210 |

## Repository layout

```
agris.py                        Python tracking app (GUI, detection, UDP sender)
AGRIS_AiThinker/
  AGRIS_AiThinker.ino           AI-Thinker firmware (MJPEG stream + UDP + servo/laser)
AGRIS_ESP32S3/
  AGRIS_ESP32S3.ino             ESP32-S3 alternate board target (same role as AiThinker)
ESP32S1/
  ESP32S1.ino                   Mecanum drive firmware (DualShock4 → motors)
CAD_Design/
  part-1.dxf                    Laser-cut frame parts
  part-2.dxf
  part-3.dxf
```

## Software dependencies

```
pip install opencv-python numpy pillow requests
```

Arduino libraries: `esp32-camera`, `ESP32Servo`, `AsyncUDP`, `Bluepad32`

## Detected shapes

The vision pipeline uses adaptive thresholding and contour classification:

- Rectangle
- Square
- Circle
- Plus cross (+)
- X cross (×)

Target priority and per-shape enable/disable are configurable at runtime in the GUI.

## Communication protocol

UDP packet sent from `agris.py` to the ESP32:

```
PAN:xxxx,TILT:xxxx,LASER:x
```

Values are microseconds for the servo signal. Laser is `0` or `1`.

## Servo ranges

| Axis | Min | Center | Max |
|---|---|---|---|
| Pan | 500 µs | 1632 µs | 2500 µs |
| Tilt | 1050 µs | 1500 µs | 2200 µs |

## AI-Thinker GPIO assignments

| Function | GPIO |
|---|---|
| Servo pan | 12 |
| Servo tilt | 13 |
| Laser | 2 |
| Camera PWDN | 32 |

## Notes

- Camera is fixed to the frame. Only the laser/servo assembly moves.
- Tracking uses one-shot absolute positioning — servo position is calculated directly from pixel error and camera FOV, not accumulated incrementally.
- LASER pin is held LOW during boot to prevent the laser firing on startup.
- AI-Thinker uses `CAMERA_FB_IN_DRAM` with `fb_count=1` (no PSRAM on this board).
- ESP32-S3 sketch is an alternate board target with the same firmware role as the AI-Thinker.
