# AGV

Autonomous Guided Vehicle on Jetson Orin Nano: camera line-following with
LiDAR safety and STM32 motor control.

See **[AGV_README.md](AGV_README.md)** for the full design, bring-up notes,
testing plan, and architecture decisions. The short version:

## Quick start

```bash
# 1. Install the camera-capable OpenCV (apt, has GStreamer; do not use pip)
sudo apt install python3-opencv

# 2. Wire the IMX219 to CAM0, LiDAR to USB, STM32 to /dev/ttyTHS1

# 3. Run the control loop (orchestrator)
python3 agv_main.py

# 4. In another terminal, run the read-only web dashboard
. .venv-dashboard/bin/activate
python web.py
# open http://<jetson-lan-ip>:5000
```

## Project layout

| File | Role |
|---|---|
| `agv_main.py` | Thin orchestrator: ties modules together in the main loop |
| `config.py` | Tunable constants and camera-backend selection |
| `camera.py` | Camera abstraction: argus / v4l2 / libcamera / fake |
| `yellow_line.py` | Yellow-line detection (HSV segmentation + contour centroid) |
| `lidar.py` | RPLIDAR A1 wrapper (background thread, thread-safe snapshots) |
| `stm32.py` | STM32 motor-controller (UART, ACK + encoder + version) |
| `decision.py` | Pure function: combine camera + lidar → motor command |
| `telemetry.py` | Atomic on-disk publisher (status.json + frame.jpg) |
| `web.py` | Flask dashboard (read-only) |
| `dashboard.py` | Legacy dashboard (kept for reference) |
| `camera_test.py` | Standalone camera bring-up tool |
| `manual_motor_test.py` | Manual STM32 test harness |
| `firmware/` | STM32 firmware: `main_v3.c`, `main.h`, HAL config, Makefile |

## Current status (2026-10-15)

- **Camera:** working on Jetson Orin Nano + JetPack R39.2 (IMX219 on CAM0,
  `1280×720 @ 60 fps` via `nvarguscamerasrc`).
- **LiDAR:** code ready; `/dev/ttyUSB0` not yet attached.
- **STM32:** code ready; `/dev/ttyTHS1` permission not yet resolved.
- **Dashboard:** implemented and runs; hardware validation pending.
- **Vehicle integration:** pending.

## More docs

- [AGV_README.md](AGV_README.md) — full design, milestones, bring-up notes
- [WEB_DASHBOARD_PLAN.md](WEB_DASHBOARD_PLAN.md) — dashboard design
- [CAMERA_DIAGNOSTIC_REPORT.md](CAMERA_DIAGNOSTIC_REPORT.md) — 2026-10-09
  bring-up failure and 2026-10-15 resolution
