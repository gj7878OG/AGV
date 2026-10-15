"""Central configuration for the AGV control program.

All tunable constants live here so behavior can be adjusted without
editing the main control loop. Values that depend on the physical
vehicle, floor material, or lighting should be re-calibrated on the
target environment.
"""

from __future__ import annotations

import os

# ── Serial ports ───────────────────────────────────────────────────────────
LIDAR_PORT = '/dev/ttyUSB0'           # RPLIDAR A1 USB
STM32_PORT = '/dev/ttyTHS1'           # Jetson UART1 to STM32
BAUD_RATE = 115200

# ── LiDAR safety zones (millimeters) ──────────────────────────────────────
AGV_HALF_WIDTH = 350                  # Corridor half-width (700 mm total)
STOP_DISTANCE = 1200                  # Emergency stop zone (< 1.2 m)
SLOW_DISTANCE = 2500                  # Slow-down zone (1.2 m – 2.5 m)

# ── Camera line-following ──────────────────────────────────────────────────
STEER_DEADZONE = 80                   # Pixels from center before steering
MIN_CONTOUR_AREA = 500                # Reject small noise contours
MORPHOLOGY_KERNEL = (5, 5)            # Kernel for open/close ops on mask
# HSV range for the yellow navigation line. Calibrate against actual tape,
# floor color, and lighting — these are starting points only.
YELLOW_HSV_LOWER = (20, 100, 100)
YELLOW_HSV_LOWER_2 = (170, 100, 100)  # wraparound for red-leaning yellows (rare)
YELLOW_HSV_UPPER = (35, 255, 255)

# ── Camera backend choice ──────────────────────────────────────────────────
# AGV_CAMERA_BACKEND ∈ {argus, v4l2, libcamera, fake}.
# - 'argus'    : NVIDIA legacy Argus pipeline (nvarguscamerasrc). Best ISP.
# - 'v4l2'     : NVIDIA V4L2 path (nvv4l2camerasrc). Lower latency.
# - 'libcamera': libcamera + picamera2. Works without NVIDIA stack.
# - 'fake'     : generates a synthetic yellow line for offline testing.
CAMERA_BACKEND = os.environ.get('AGV_CAMERA_BACKEND', 'argus').lower()
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 60                       # IMX219 exposes 1280x720 only at 60 fps
CAMERA_SENSOR_ID = 0                  # IMX219 CSI sensor-id

# Legacy Argus pipeline (JetPack ≤ 5, still supported on JetPack 6).
_ARGUS_PIPELINE = (
    "nvarguscamerasrc sensor-id={sensor} ! "
    "video/x-raw(memory:NVMM), width={w}, height={h}, "
    "format=NV12, framerate={fps}/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
).format(sensor=CAMERA_SENSOR_ID, w=CAMERA_WIDTH, h=CAMERA_HEIGHT, fps=CAMERA_FPS)

# Modern V4L2 path (JetPack 6 recommended).
_V4L2_PIPELINE = (
    "nvv4l2camerasrc sensor-id={sensor} ! "
    "video/x-raw(memory:NVMM), width={w}, height={h}, "
    "format=NV12, framerate={fps}/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
).format(sensor=CAMERA_SENSOR_ID, w=CAMERA_WIDTH, h=CAMERA_HEIGHT, fps=CAMERA_FPS)

# libcamera path (no NVIDIA stack required; works on stock Linux).
_LIBCAMERA_PIPELINE = (
    "libcamerasrc camera-name={sano} ! "
    "video/x-raw, width={w}, height={h}, framerate={fps}/1 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
).format(sano=CAMERA_SENSOR_ID, w=CAMERA_WIDTH, h=CAMERA_HEIGHT, fps=CAMERA_FPS)

CAMERA_PIPELINES: dict[str, str] = {
    'argus':     _ARGUS_PIPELINE,
    'v4l2':      _V4L2_PIPELINE,
    'libcamera': _LIBCAMERA_PIPELINE,
}

# ── Telemetry / dashboard ─────────────────────────────────────────────────
TELEMETRY_DIR = os.environ.get('AGV_DASHBOARD_DIR', '/tmp/agv-dashboard')
LOG_LIMIT = 500                       # Max telemetry log entries retained
SHOW_WINDOW = os.environ.get(
    'AGV_SHOW_WINDOW', '1' if os.environ.get('DISPLAY') else '0'
).lower() in ('1', 'true', 'yes', 'on')
DASHBOARD_HOST = os.environ.get('AGV_DASHBOARD_HOST', '0.0.0.0')
DASHBOARD_HTTP_PORT = int(os.environ.get('AGV_DASHBOARD_PORT', '5000'))
DASHBOARD_WS_PORT = int(os.environ.get('AGV_DASHBOARD_WS_PORT', '5001'))

# ── Watchdog / health ─────────────────────────────────────────────────────
STM32_ACK_TIMEOUT_MS = 500             # Mark STM32 disconnected after this
STM32_HEARTBEAT_INTERVAL_MS = 200      # Python sends 'alive' ping cadence
ENCODER_TELEMETRY_INTERVAL_MS = 100   # STM32 encoder report cadence
LIDAR_BUCKET_MM = 250                  # Group LiDAR distance logs by this width
LOG_FILE = os.environ.get(
    'AGV_LOG_FILE', os.path.join(TELEMETRY_DIR, 'agv.log')
)


def get_camera_pipeline(backend: str | None = None) -> str | None:
    """Return the GStreamer pipeline string for the chosen camera backend.

    Returns None for 'fake' (caller should instantiate a synthetic source).
    """
    name = (backend or CAMERA_BACKEND).lower()
    return CAMERA_PIPELINES.get(name)