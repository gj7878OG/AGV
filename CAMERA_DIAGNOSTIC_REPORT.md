# Camera Startup Diagnostic Report

**Run date:** 2026-10-09  
**Camera:** Sony IMX219 CSI  
**Capture path:** Jetson Argus through GStreamer and OpenCV

## What happened

The controller attempted to start camera capture before connecting to the
STM32 or starting the LiDAR. It opened the IMX219 GStreamer pipeline first.

The controller was launched with `AGV_SHOW_WINDOW=0`, which disables the local
OpenCV preview window. The dashboard could still display the feed, but only if
the controller received and published camera frames.

It received no frames. The runtime output included:

- `No cameras available`
- `Camera running headlessly`
- `Frame read failed; stopping AGV`

The controller treated the pipeline as open, then stopped when its first frame
read failed. In [`agv_main.py`](agv_main.py), the startup check calls
`cap.isOpened()` before printing that the camera opened; it does not verify
that a frame is available until the later `cap.read()` call. In this run, the
“opened” message was misleading because the pipeline had not produced a usable
image.

## Camera diagnostics

The diagnostics collected after the run showed:

- `nvargus-daemon` was active. This confirms the service was running, but not
  that a sensor was available.
- Argus reported that its V4L2 camera device was unavailable and that it could
  not open a sensor.
- IMX219 probes on I²C buses 7 and 8 failed with error `-121`.
- No `/dev/video*` device was present; `/dev/media0` was present.

Together, these findings indicate that the camera sensor was not responding
or was not exposed as a usable capture device to the camera driver. Likely
causes include the CSI ribbon connection or orientation, the camera module,
or camera/driver initialization. The historical IMX219 notes in
[`AGV_README.md`](AGV_README.md) describe similar I²C and Argus failures; the
current diagnostics independently show the same type of low-level problem.

## Other device failures

The STM32 connection also failed because access to `/dev/ttyTHS1` was denied.
The LiDAR connection failed because `/dev/ttyUSB0` was missing. These failures
prevent normal AGV operation, but they did not prevent the camera attempt. The
controller entered camera-only mode, then exited because it could not read a
camera frame.

The standalone [`camera_test.py`](camera_test.py) uses the same GStreamer
pipeline. It checks `isOpened()` before trying to read a frame, so it may show
the same misleading startup message before reporting a frame-read error.

## Recommended next steps

1. Fully power off the Jetson before handling the CSI ribbon cable.
2. Check the cable for damage, then reseat it at both ends. Confirm its
   orientation and that both connector locks are closed.
3. Reboot and check whether a video device appears and whether a native
   `nvarguscamerasrc` preview can produce frames.
4. Once native capture works, retry `camera_test.py`, then check the AGV
   dashboard feed.

The local preview window also requires `AGV_SHOW_WINDOW=1` and an available
graphical display. With `AGV_SHOW_WINDOW=0`, capture can run headlessly, but no
local OpenCV window is shown.
