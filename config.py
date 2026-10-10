"""Central configuration for the AGV control program.

All tunable constants live here so behavior can be adjusted without
editing the main control loop. Values that depend on the physical
vehicle, floor material, or lighting should be re-calibrated on the
target environment.
"""

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
YELLOW_HSV_UPPER = (35, 255, 255)

# ── Camera pipeline (IMX219 on Jetson) ────────────────────────────────────
CAMERA_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, "
    "format=NV12, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
)

# ── Telemetry / dashboard ─────────────────────────────────────────────────
TELEMETRY_DIR = os.environ.get('AGV_DASHBOARD_DIR', '/tmp/agv-dashboard')
LOG_LIMIT = 500                       # Max telemetry log entries retained
SHOW_WINDOW = os.environ.get(
    'AGV_SHOW_WINDOW', '1' if os.environ.get('DISPLAY') else '0'
).lower() in ('1', 'true', 'yes', 'on')

# ── Watchdog / health ─────────────────────────────────────────────────────
STM32_ACK_TIMEOUT_MS = 500             # Mark STM32 disconnected after this
STM32_HEARTBEAT_INTERVAL_MS = 200      # Python sends 'alive' ping cadence
ENCODER_TELEMETRY_INTERVAL_MS = 100   # STM32 encoder report cadence
LOG_FILE = os.environ.get(
    'AGV_LOG_FILE', os.path.join(TELEMETRY_DIR, 'agv.log')
)