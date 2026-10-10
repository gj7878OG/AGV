#!/usr/bin/env python3
"""
AGV Main Brain — Unified Control System
========================================
Runs OpenCV camera line following as PRIMARY control
with LiDAR as a SAFETY-ONLY backup on Jetson Nano.

Control Logic:
  PRIMARY (Camera — OpenCV):
    - Camera detects yellow line → decides L / R / F / S
    - This is the main brain of the AGV

  SAFETY ONLY (LiDAR):
    - LiDAR STOP  → Emergency stop ONLY if obstacle < 1.2m
    - LiDAR SLOW  → Reduce speed only when going straight (F→W)
    - LiDAR does NOT override camera steering (L/R)
    - If no obstacle → camera has full control

Wiring:
  RPLIDAR A1    → Jetson Nano USB (/dev/ttyUSB0)
  IMX219 Camera → Jetson Nano CSI ribbon
  Jetson Pin 8  (TX) → STM32 PA10 (RX)
  Jetson Pin 10 (RX) → STM32 PA9  (TX)
  Jetson Pin 6  (GND) → STM32 GND

Dependencies (install on Jetson Nano):
  pip3 install pyserial rplidar-roboticia
  NumPy and OpenCV must also be available in the Jetson Python environment.

Dashboard (optional, in another terminal):
  pip3 install -r requirements-dashboard.txt
  python3 dashboard.py

Usage:
  python3 agv_main.py
"""

import cv2
import numpy as np
import serial
import time
import math
import threading
import json
import logging
import os
import tempfile
from collections import deque
from rplidar import RPLidar, RPLidarException

from config import (
    LIDAR_PORT, STM32_PORT, BAUD_RATE,
    AGV_HALF_WIDTH, STOP_DISTANCE, SLOW_DISTANCE,
    STEER_DEADZONE, MIN_CONTOUR_AREA, MORPHOLOGY_KERNEL,
    YELLOW_HSV_LOWER, YELLOW_HSV_UPPER,
    CAMERA_PIPELINE, TELEMETRY_DIR, LOG_LIMIT, SHOW_WINDOW,
    STM32_ACK_TIMEOUT_MS, STM32_HEARTBEAT_INTERVAL_MS,
    ENCODER_TELEMETRY_INTERVAL_MS, LOG_FILE,
)

# ─────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────
os.makedirs(os.path.dirname(LOG_FILE) or '.', exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger('agv')
# Lower volume from third-party libs.
logging.getLogger('werkzeug').setLevel(logging.WARNING)

# ─────────────────────────────────────────────
#  SHARED STATE (thread-safe)
# ─────────────────────────────────────────────

# LiDAR thread writes this, main loop reads it
lidar_state = {
    'status': 'CLEAR',       # 'CLEAR', 'SLOW', 'STOP'
    'distance': 0,           # Closest obstacle distance in mm
    'connected': False,
    'error': None,
    'running': True,         # Set False to kill the thread
}
lidar_lock = threading.Lock()
telemetry_lock = threading.Lock()
telemetry_logs = deque(maxlen=LOG_LIMIT)
telemetry_log_id = 0
telemetry_condition = threading.Condition()
telemetry_pending = None
telemetry_stopping = False
telemetry_thread = None

# STM32 reader thread writes these; main loop reads them.
# `last_ack_monotonic` is updated every time a 'A' byte comes back from
# the STM32. The main loop compares (now - last_ack_monotonic) against
# STM32_ACK_TIMEOUT_MS to decide whether to trust the serial link.
stm32_state = {
    'last_ack_monotonic': 0.0,
    'encoder_left': 0,
    'encoder_right': 0,
    'running': True,
    'connected': False,
}
stm32_lock = threading.Lock()
stm32_reader_thread = None

# Results from the startup sensor probe. Each entry is a dict:
#   {device, status, port, error, latency_ms, [serial]}
# `status` is 'OK', 'FAILED', 'ERROR', or 'NO_FRAMES'.
sensor_probes = []
last_error = None  # Most recent uncaught exception traceback (str).


def _atomic_write(path, data):
    """Replace a telemetry file atomically so readers never see partial data."""
    os.makedirs(TELEMETRY_DIR, exist_ok=True)
    fd, temp_path = tempfile.mkstemp(prefix='.agv-', dir=TELEMETRY_DIR)
    try:
        with os.fdopen(fd, 'wb') as temp_file:
            temp_file.write(data)
        os.replace(temp_path, path)
    finally:
        if os.path.exists(temp_path):
            os.unlink(temp_path)


# Log categories — surfaced as separate dashboard panels.
LOG_CAT_SYSTEM  = 'SYSTEM'   # startup, probe, config, exceptions
LOG_CAT_CAMERA  = 'CAMERA'   # vision, line detection, frame errors
LOG_CAT_LIDAR   = 'LIDAR'    # obstacle transitions, distance buckets, errors
LOG_CAT_STM32   = 'STM32'    # ACK, encoder, watchdog, command send, errors

LOG_CATEGORIES = (LOG_CAT_SYSTEM, LOG_CAT_CAMERA, LOG_CAT_LIDAR, LOG_CAT_STM32)


def _normalize_log_message(message, category):
    """Return ([{id, timestamp, category, message}, ...], default_category).

    `message` may be a string, a list of strings, or a list of
    (category, message) tuples. Strings get the default category.
    """
    if message is None:
        return [], category
    if isinstance(message, (list, tuple)):
        items = message
    else:
        items = [message]
    out = []
    for item in items:
        if item is None:
            continue
        if isinstance(item, tuple) and len(item) == 2 and item[0] in LOG_CATEGORIES:
            cat, text = item
        else:
            cat, text = category, str(item)
        out.append({'category': cat, 'message': text})
    return out, category


def publish_telemetry(state, frame=None, log_message=None, log_category=LOG_CAT_SYSTEM):
    """Queue read-only state and frame; disk I/O stays off the control loop.

    `log_message` may be:
      - a string (gets `log_category`)
      - a list of strings (each gets `log_category`)
      - a list of (category, text) tuples for mixed-category batches
    """
    global telemetry_log_id, telemetry_pending

    with telemetry_lock:
        entries, _ = _normalize_log_message(log_message, log_category)
        for entry in entries:
            telemetry_log_id += 1
            telemetry_logs.append({
                'id': telemetry_log_id,
                'timestamp': time.time(),
                'category': entry['category'],
                'message': entry['message'],
            })

        payload = dict(state)
        payload.update({
            'updated_at': time.time(),
            'logs': list(telemetry_logs),
        })

    with telemetry_condition:
        # Keep only the newest frame and status if the writer falls behind.
        pending_frame = frame if frame is not None else (
            telemetry_pending[1] if telemetry_pending else None
        )
        telemetry_pending = (payload, pending_frame)
        telemetry_condition.notify()


def telemetry_writer():
    """Write queued updates at up to 5 fps without delaying motor decisions."""
    global telemetry_pending
    while True:
        with telemetry_condition:
            while telemetry_pending is None and not telemetry_stopping:
                telemetry_condition.wait()
            if telemetry_pending is None and telemetry_stopping:
                return
            payload, frame = telemetry_pending
            telemetry_pending = None

        try:
            if frame is not None:
                encoded, jpeg = cv2.imencode(
                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if encoded:
                    _atomic_write(os.path.join(TELEMETRY_DIR, 'frame.jpg'), jpeg.tobytes())
            encoded_state = json.dumps(payload).encode('utf-8')
            _atomic_write(os.path.join(TELEMETRY_DIR, 'status.json'), encoded_state)
        except (OSError, cv2.error, TypeError, ValueError) as exc:
            log.info(f"[Dashboard] Telemetry publish failed: {exc}")
        if frame is not None:
            time.sleep(0.2)


def start_telemetry_writer():
    global telemetry_thread
    if telemetry_thread is None:
        telemetry_thread = threading.Thread(
            target=telemetry_writer, name='dashboard-telemetry', daemon=True
        )
        telemetry_thread.start()


def stop_telemetry_writer():
    global telemetry_stopping
    with telemetry_condition:
        telemetry_stopping = True
        telemetry_condition.notify_all()
    if telemetry_thread is not None:
        telemetry_thread.join(timeout=2)


def restore_telemetry_logs():
    """Keep event IDs and recent history continuous across control restarts."""
    global telemetry_log_id
    try:
        with open(os.path.join(TELEMETRY_DIR, 'status.json'), 'r', encoding='utf-8') as status_file:
            previous = json.load(status_file)
        for entry in previous.get('logs', [])[-LOG_LIMIT:]:
            # Backfill `category` for entries written before categorization.
            if 'category' not in entry:
                entry['category'] = LOG_CAT_SYSTEM
            telemetry_logs.append(entry)
            telemetry_log_id = max(telemetry_log_id, int(entry.get('id', 0)))
    except (OSError, ValueError, TypeError):
        pass


# ─────────────────────────────────────────────
#  STARTUP SENSOR PROBE
# ─────────────────────────────────────────────

def _try_open_port(port, baud=BAUD_RATE, timeout=1.0):
    """Try to open a serial port; return (handle, error_message)."""
    try:
        handle = serial.Serial(port, baud, timeout=timeout)
        return handle, None
    except Exception as e:
        return None, f'{type(e).__name__}: {e}'


def probe_sensors():
    """Probe each known sensor and return a list of result dicts.

    Each result has device/port/status/error/latency_ms. Status is one of:
      'OK'         — opened and (where applicable) returned valid data
      'FAILED'     — port not reachable / device did not respond
      'ERROR'      — unexpected exception during probe
      'NO_FRAMES'  — opened but did not yield a usable frame (camera only)
    The probe never raises; every device reports independently so a single
    failure cannot mask the others.
    """
    probes = []

    # ── Camera (CSI ribbon — no traditional port number) ──
    t0 = time.monotonic()
    camera_status = 'FAILED'
    camera_error = None
    try:
        cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                camera_status = 'OK'
            else:
                camera_status = 'NO_FRAMES'
                camera_error = 'Pipeline opened but produced no frames (check sensor link)'
            cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER).release()
        else:
            camera_error = 'cv2.VideoCapture.isOpened() returned False (check CSI ribbon)'
    except Exception as e:
        camera_status = 'ERROR'
        camera_error = f'{type(e).__name__}: {e}'
    probes.append({
        'device': 'camera',
        'port': 'CSI (IMX219 ribbon)',
        'status': camera_status,
        'error': camera_error,
        'latency_ms': round((time.monotonic() - t0) * 1000, 1),
    })
    log.info("[Probe] Camera: %s — %s", camera_status, camera_error or 'OK')

    # ── STM32 UART ──
    t0 = time.monotonic()
    handle, err = _try_open_port(STM32_PORT)
    stm32_status = 'OK' if handle else 'FAILED'
    if handle:
        handle.close()
    probes.append({
        'device': 'stm32',
        'port': STM32_PORT,
        'status': stm32_status,
        'error': err,
        'latency_ms': round((time.monotonic() - t0) * 1000, 1),
    })
    log.info("[Probe] STM32 (%s): %s — %s", STM32_PORT, stm32_status, err or 'OK')

    # ── LiDAR USB ──
    t0 = time.monotonic()
    lidar_status = 'FAILED'
    lidar_error = None
    lidar_serial = None
    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_RATE, timeout=3)
        info = lidar.get_info()
        lidar_serial = info.get('serialnumber')
        lidar_status = 'OK'
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except RPLidarException as e:
        lidar_error = f'RPLidarException: {e}'
    except Exception as e:
        lidar_status = 'ERROR'
        lidar_error = f'{type(e).__name__}: {e}'
    probes.append({
        'device': 'lidar',
        'port': LIDAR_PORT,
        'status': lidar_status,
        'error': lidar_error,
        'serial': lidar_serial,
        'latency_ms': round((time.monotonic() - t0) * 1000, 1),
    })
    log.info("[Probe] LiDAR (%s): %s — %s", LIDAR_PORT, lidar_status, lidar_error or 'OK')

    return probes


# ─────────────────────────────────────────────
#  STM32 SERIAL
# ─────────────────────────────────────────────

def stm32_reader_thread_func(stm):
    """Consume ACK ('A') and encoder telemetry ('E' + 4 bytes) from STM32.

    Runs as a daemon; updates shared `stm32_state` under `stm32_lock`.
    A break in the ACK stream means the STM32 may have stopped responding.
    """
    global stm32_state
    log.info("[STM32] reader thread started")
    pending_encoder_bytes = 0
    encoder_buffer = bytearray()

    while True:
        with stm32_lock:
            if not stm32_state['running']:
                break

        try:
            byte = stm.read(1)
            if not byte:
                continue

            if pending_encoder_bytes > 0:
                encoder_buffer.append(byte[0])
                pending_encoder_bytes -= 1
                if pending_encoder_bytes == 0 and len(encoder_buffer) == 4:
                    left = (encoder_buffer[0] << 8) | encoder_buffer[1]
                    right = (encoder_buffer[2] << 8) | encoder_buffer[3]
                    with stm32_lock:
                        stm32_state['encoder_left'] = left
                        stm32_state['encoder_right'] = right
                    encoder_buffer.clear()
                continue

            if byte == b'A':
                with stm32_lock:
                    stm32_state['last_ack_monotonic'] = time.monotonic()
                    stm32_state['connected'] = True
            elif byte == b'E':
                pending_encoder_bytes = 4
                encoder_buffer.clear()

        except (serial.SerialException, OSError) as exc:
            log.warning("[STM32] reader disconnected: %s", exc)
            with stm32_lock:
                stm32_state['connected'] = False
            time.sleep(0.5)

    log.info("[STM32] reader thread exiting")


def connect_stm32():
    """Connect to STM32 via UART and start the ACK/encoder reader thread."""
    global stm32_reader_thread
    try:
        stm = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)
        with stm32_lock:
            stm32_state['running'] = True
            stm32_state['last_ack_monotonic'] = time.monotonic()
            stm32_state['connected'] = False
        stm32_reader_thread = threading.Thread(
            target=stm32_reader_thread_func, args=(stm,),
            name='stm32-reader', daemon=True,
        )
        stm32_reader_thread.start()
        log.info("STM32 connected on %s", STM32_PORT)
        return stm
    except Exception as e:
        log.error("Cannot connect STM32 on %s: %s", STM32_PORT, e)
        log.error("AGV will not move without STM32 connection.")
        return None


def send_to_stm32(stm, cmd_byte):
    """Send a command byte and report whether the serial write succeeded."""
    if stm:
        try:
            stm.write(cmd_byte)
            return True
        except serial.SerialException:
            return False
    return False


# ─────────────────────────────────────────────
#  LIDAR THREAD
# ─────────────────────────────────────────────

def lidar_thread_func():
    """Background thread: continuously scan with LiDAR and update shared state."""
    global lidar_state

    log.info(f"[LiDAR] Connecting to RPLIDAR on {LIDAR_PORT}...")
    lidar = None

    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_RATE, timeout=3)
        lidar.clean_input()
        lidar.reset()
        time.sleep(1.5)

        info = lidar.get_info()
        log.info(f"[LiDAR]  Connected | S/N: {info.get('serialnumber')}")
        log.info(f"[LiDAR]    Stop: <{STOP_DISTANCE/1000:.1f}m | Slow: <{SLOW_DISTANCE/1000:.1f}m | Width: {AGV_HALF_WIDTH*2/1000:.1f}m")
        with lidar_lock:
            lidar_state['connected'] = True
            lidar_state['error'] = None

        for scan in lidar.iter_scans(max_buf_meas=500):
            # Check if main thread wants us to stop
            with lidar_lock:
                if not lidar_state['running']:
                    break

            closest_stop = float('inf')
            closest_slow = float('inf')

            for quality, angle_deg, dist_mm in scan:
                if dist_mm == 0:
                    continue

                theta_rad = math.radians(angle_deg)
                y = dist_mm * math.cos(theta_rad)
                x = dist_mm * math.sin(theta_rad)

                if abs(x) <= AGV_HALF_WIDTH and y > 0:
                    if y <= STOP_DISTANCE:
                        closest_stop = min(closest_stop, y)
                    elif y <= SLOW_DISTANCE:
                        closest_slow = min(closest_slow, y)

            # Update shared state
            with lidar_lock:
                if closest_stop < float('inf'):
                    lidar_state['status'] = 'STOP'
                    lidar_state['distance'] = closest_stop
                elif closest_slow < float('inf'):
                    lidar_state['status'] = 'SLOW'
                    lidar_state['distance'] = closest_slow
                else:
                    lidar_state['status'] = 'CLEAR'
                    lidar_state['distance'] = 0

    except RPLidarException as e:
        log.error(f"[LiDAR]  Error: {e}")
        with lidar_lock:
            lidar_state['connected'] = False
            lidar_state['error'] = str(e)
    except Exception as e:
        log.error(f"[LiDAR]  Unexpected error: {e}")
        with lidar_lock:
            lidar_state['connected'] = False
            lidar_state['error'] = str(e)
    finally:
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        with lidar_lock:
            lidar_state['connected'] = False
        log.info("[LiDAR] Disconnected.")


# ─────────────────────────────────────────────
#  CAMERA + DECISION LOOP (main thread)
# ─────────────────────────────────────────────

def run_agv():
    global lidar_state, sensor_probes, last_error

    restore_telemetry_logs()
    start_telemetry_writer()

    log.info("=" * 55)
    log.info("  AGV MAIN BRAIN — Unified Control System")
    log.info("  LiDAR + Camera → Jetson Nano → STM32")
    log.info("=" * 55)
    log.info()
    with lidar_lock:
        lidar_state['running'] = True
        lidar_state['status'] = 'CLEAR'
        lidar_state['distance'] = 0
        lidar_state['connected'] = False
        lidar_state['error'] = None

    # Probe every sensor up front so the dashboard reflects the full
    # wiring picture before any main-loop decisions are made.
    sensor_probes = probe_sensors()

    publish_telemetry({
        'running': False,
        'camera_connected': False,
        'stm32_connected': False,
        'camera_direction': 'STARTING',
        'camera_command': 'S',
        'lidar_status': 'CLEAR',
        'lidar_connected': False,
        'lidar_error': None,
        'lidar_distance_mm': 0,
        'final_command': 'S',
        'sensor_probes': sensor_probes,
        'last_error': None,
    }, log_message=[
        ('SYSTEM', 'AGV control process starting'),
        *[(LOG_CAT_SYSTEM, f"Probe {p['device']}: {p['status']} on {p['port']}")
          for p in sensor_probes],
    ])

    # 1. Open the camera first. Its preview and telemetry remain available
    # even when the motor controller or LiDAR is disconnected.
    log.info("\n[Camera] Opening IMX219...")
    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        # Camera missing — stay alive so the dashboard keeps reflecting
        # status, and the operator can see what failed instead of the
        # process silently exiting.
        global last_error
        last_error = 'Camera could not be opened (CSI ribbon or sensor fault)'
        log.error("[Camera]  Could not open IMX219 — entering degraded mode")
        publish_telemetry({
            'running': True, 'camera_connected': False,
            'stm32_connected': False, 'camera_direction': 'UNAVAILABLE',
            'camera_command': 'S', 'lidar_status': 'UNKNOWN',
            'lidar_connected': False, 'lidar_error': None,
            'lidar_distance_mm': 0, 'final_command': 'S',
            'sensor_probes': sensor_probes, 'last_error': last_error,
        }, log_message='Camera unavailable — degraded mode', log_category=LOG_CAT_CAMERA)
        cap = None
    else:
        log.info("[Camera]  IMX219 opened at 1280x720 @30fps")

    # Connect to the STM32, but keep the camera running without it.
    stm = connect_stm32()
    stm32_connected = bool(stm and send_to_stm32(stm, b'S'))
    if not stm32_connected:
        log.info("[STM32] Unavailable; camera-only mode, motor command held at S.")
        if stm:
            stm.close()
            stm = None
    publish_telemetry({
        'running': False,
        'camera_connected': False,
        'stm32_connected': stm32_connected,
        'camera_direction': 'STARTING',
        'camera_command': 'S',
        'lidar_status': 'CLEAR',
        'lidar_connected': False,
        'lidar_error': None,
        'lidar_distance_mm': 0,
        'final_command': 'S',
        'sensor_probes': sensor_probes,
        'last_error': last_error,
    }, log_message=(
        'STM32 connected; initial stop command sent' if stm32_connected
        else 'STM32 command write failed during startup'
    ), log_category=LOG_CAT_STM32)

    # 2. Start LiDAR in the background; do not block camera capture on it.
    lidar_t = threading.Thread(target=lidar_thread_func, daemon=True)
    lidar_t.start()
    camera_connected = bool(cap and cap.isOpened())
    if SHOW_WINDOW:
        log.info("\n Camera running — Press Q in camera window to stop\n")
    else:
        log.info("\n Camera running headlessly — Press Ctrl+C to stop\n")

    last_cmd = None
    last_speed_sent = None  # Last STM32 speed byte ('1'..'5'), None until sent
    slow_requested = False   # True when LiDAR says LiDAR is in SLOW zone and we're going forward
    last_camera_state = None
    last_camera_connection = None
    last_lidar_state = None
    last_lidar_connection = None
    last_stm32_connection = None
    last_final_cmd = None
    stop_reason = 'camera stream ended'

    try:
        while True:
            # Inner per-iteration guard. Any uncaught exception here is
            # logged with full traceback and pushed to the dashboard as
            # `last_error`, then the loop continues — the camera feed
            # stays alive in degraded mode instead of dying.
            try:
                frame = None
                direction = 'UNAVAILABLE' if cap is None else 'STARTING'
                camera_cmd = 'S'

                if cap is not None:
                    ret = False
                    try:
                        ret, frame = cap.read()
                    except cv2.error as exc:
                        camera_connected = False
                        last_error = f'cv2.error during read: {exc}'
                        log.error('[Camera]  Frame read exception: %s', exc)
                    except Exception as exc:
                        camera_connected = False
                        last_error = f'{type(exc).__name__}: {exc}'
                        log.error('[Camera]  Unexpected read error: %s', exc)

                    if not ret:
                        camera_connected = False
                        if last_error is None:
                            last_error = 'cap.read() returned False (sensor or pipeline fault)'
                        log.warning('[Camera]  Frame read failed; staying alive in degraded mode')

                # ── Read LiDAR state ──
                with lidar_lock:
                    obstacle_status = lidar_state['status']
                    obstacle_dist = lidar_state['distance']
                    lidar_connected = lidar_state['connected']
                    lidar_error = lidar_state['error']

                # ── Camera: detect yellow line ──
                if frame is not None:
                    try:
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        lower_yellow = np.array(YELLOW_HSV_LOWER)
                        upper_yellow = np.array(YELLOW_HSV_UPPER)
                        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                        kernel = np.ones(MORPHOLOGY_KERNEL, np.uint8)
                        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                        contours, _ = cv2.findContours(
                            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )

                        output = frame.copy()
                        center_x = frame.shape[1] // 2

                        if contours:
                            contour = max(contours, key=cv2.contourArea)
                            area = cv2.contourArea(contour)

                            if area > MIN_CONTOUR_AREA:
                                M = cv2.moments(contour)
                                if M["m00"] != 0:
                                    cx = int(M["m10"] / M["m00"])
                                    cy = int(M["m01"] / M["m00"])
                                    error = cx - center_x

                                    if error < -STEER_DEADZONE:
                                        direction = "LEFT"
                                        camera_cmd = 'L'
                                    elif error > STEER_DEADZONE:
                                        direction = "RIGHT"
                                        camera_cmd = 'R'
                                    else:
                                        direction = "CENTER"
                                        camera_cmd = 'F'

                                    cv2.drawContours(output, [contour], -1, (0, 255, 0), 3)
                                    cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
                                    cv2.line(output, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)
                    except cv2.error as exc:
                        last_error = f'cv2.error in line detection: {exc}'
                        log.error('[Vision]  %s', last_error)
                    except Exception as exc:
                        last_error = f'{type(exc).__name__} in line detection: {exc}'
                        log.error('[Vision]  %s', last_error)

                # ─────────────────────────────────────────────
                #  DECISION ENGINE
                #  PRIMARY:  Camera (OpenCV yellow line following)
                #  SAFETY:   LiDAR (emergency stop only)
                # ─────────────────────────────────────────────

                # Camera is ALWAYS the primary decision maker
                final_cmd = camera_cmd
                status_text = f"Line: {direction}"
                status_color = (0, 255, 0)  # Green

                # LiDAR safety override — ONLY when about to crash
                if obstacle_status == 'STOP':
                    final_cmd = 'S'
                    status_text = f"⚠ SAFETY STOP | Obstacle {obstacle_dist/1000:.2f}m"
                    status_color = (0, 0, 255)
                elif obstacle_status == 'SLOW' and camera_cmd == 'F':
                    # Slow forward: send speed byte '2' (40%) before 'F'.
                    # STM32 firmware doesn't define a 'W' command; speed is
                    # sent as a separate byte and ACK'd independently.
                    final_cmd = 'F'
                    slow_requested = True
                    status_text = f"Line: {direction} | Obstacle {obstacle_dist/1000:.2f}m (slowing)"
                    status_color = (0, 255, 255)
                else:
                    slow_requested = False

                # Camera preview stays live; motion is held at S until
                # both the STM32 and LiDAR safety sensor are available.
                if not stm32_connected or not lidar_connected:
                    final_cmd = 'S'
                    status_text = ('CAMERA ONLY | STM32 unavailable' if not stm32_connected
                                   else 'CAMERA ONLY | LiDAR unavailable')
                    status_color = (0, 0, 255)

                # STM32 watchdog — force-stop if no ACK within timeout.
                with stm32_lock:
                    ack_age_ms = (time.monotonic() - stm32_state['last_ack_monotonic']) * 1000.0
                    stm32_link_alive = ack_age_ms <= STM32_ACK_TIMEOUT_MS
                    if not stm32_link_alive and stm32_state['connected']:
                        stm32_state['connected'] = False
                    encoder_left = stm32_state['encoder_left']
                    encoder_right = stm32_state['encoder_right']
                if stm and not stm32_link_alive:
                    final_cmd = 'S'
                    status_text = f"STM32 WATCHDOG ({ack_age_ms:.0f} ms since ACK)"
                    status_color = (0, 0, 255)
                    if stm32_connected:
                        log.warning("STM32 ACK timeout (%.0f ms) — forcing STOP", ack_age_ms)
                        stm32_connected = False
                        events.append((
                            LOG_CAT_STM32,
                            f"STM32 ACK timeout ({ack_age_ms:.0f} ms) — forced STOP"
                        ))

                # Send command to STM32 (only on change).
                # When transitioning to forward, also push a speed byte first
                # if the requested speed differs from what we last sent —
                # the firmware uses the speed byte as a separate command.
                if stm and final_cmd != last_cmd:
                    target_speed = '2' if (final_cmd == 'F' and slow_requested) else '5'
                    if final_cmd == 'F' and last_speed_sent != target_speed:
                        stm32_connected = send_to_stm32(stm, target_speed.encode())
                        last_speed_sent = target_speed
                    stm32_connected = send_to_stm32(stm, final_cmd.encode())
                    log_message = [
                        (LOG_CAT_STM32,
                         f"STM32 → {final_cmd}"
                         + (f" (speed {target_speed})" if final_cmd == 'F' else '')),
                    ]
                    log.info("  → %s",
                             f"STM32: {final_cmd} | Camera: {direction} | "
                             f"LiDAR: {obstacle_status} ({obstacle_dist:.0f} mm)")
                    last_cmd = final_cmd
                else:
                    log_message = None

                # Record meaningful transitions, not every camera frame or raw
                # LiDAR distance fluctuation. Distance changes are grouped in 250mm bands.
                # `events` is a list of (category, message) tuples so each line
                # routes to the right dashboard panel.
                events = []
                camera_state = (direction, camera_cmd)
                lidar_bucket = int(obstacle_dist // 250) if obstacle_dist else 0
                current_lidar_state = (obstacle_status, lidar_bucket)
                if last_camera_state is not None and camera_state != last_camera_state:
                    events.append((
                        LOG_CAT_CAMERA,
                        f"Camera: {last_camera_state[0]} ({last_camera_state[1]}) → "
                        f"{direction} ({camera_cmd})"
                    ))
                elif last_camera_state is None:
                    events.append((LOG_CAT_CAMERA, f"Camera state: {direction} ({camera_cmd})"))
                if last_camera_connection is None:
                    events.append((
                        LOG_CAT_CAMERA,
                        'Camera connected' if camera_connected else 'Camera disconnected'
                    ))
                elif camera_connected != last_camera_connection:
                    events.append((
                        LOG_CAT_CAMERA,
                        'Camera connected' if camera_connected else 'Camera disconnected'
                    ))
                if last_stm32_connection is None:
                    events.append((
                        LOG_CAT_STM32,
                        'STM32 connected' if stm32_connected else 'STM32 disconnected'
                    ))
                elif stm32_connected != last_stm32_connection:
                    events.append((
                        LOG_CAT_STM32,
                        'STM32 connected' if stm32_connected else 'STM32 disconnected'
                    ))
                if last_lidar_connection is not None and lidar_connected != last_lidar_connection:
                    events.append((
                        LOG_CAT_LIDAR,
                        'LiDAR connected' if lidar_connected else
                        f"LiDAR disconnected{': ' + lidar_error if lidar_error else ''}"
                    ))
                elif last_lidar_connection is None:
                    events.append((
                        LOG_CAT_LIDAR,
                        'LiDAR connected' if lidar_connected else
                        f"LiDAR unavailable{': ' + lidar_error if lidar_error else ' (connecting or not detected)'}"
                    ))
                if last_lidar_state is not None and current_lidar_state != last_lidar_state:
                    old_distance = (
                        f"{last_lidar_state[1] * 250}–{(last_lidar_state[1] + 1) * 250} mm"
                        if last_lidar_state[1] else 'no obstacle'
                    )
                    new_distance = f"{obstacle_dist:.0f} mm" if obstacle_dist else 'no obstacle'
                    events.append((
                        LOG_CAT_LIDAR,
                        f"LiDAR: {last_lidar_state[0]} {old_distance} → "
                        f"{obstacle_status} {new_distance}"
                    ))
                elif last_lidar_state is None:
                    distance_text = f" at {obstacle_dist:.0f} mm" if obstacle_dist else ''
                    events.append((LOG_CAT_LIDAR, f"LiDAR state: {obstacle_status}{distance_text}"))
                if last_final_cmd is not None and final_cmd != last_final_cmd:
                    events.append((
                        LOG_CAT_STM32, f"Final command: {last_final_cmd} → {final_cmd}"
                    ))
                elif last_final_cmd is None:
                    events.append((LOG_CAT_STM32, f"Final command: {final_cmd}"))
                if events:
                    log_message = events
                last_camera_state = camera_state
                last_camera_connection = camera_connected
                last_lidar_state = current_lidar_state
                last_lidar_connection = lidar_connected
                last_stm32_connection = stm32_connected
                last_final_cmd = final_cmd

                # ── Draw HUD on camera feed ──
                if frame is not None:
                    cv2.rectangle(output, (0, 0), (frame.shape[1], 60), (0, 0, 0), -1)
                    cv2.putText(output, f"CMD: {final_cmd} | {status_text}",
                                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
                    lidar_color = {'CLEAR': (0, 255, 0), 'SLOW': (0, 255, 255), 'STOP': (0, 0, 255)}
                    cv2.circle(output, (frame.shape[1] - 40, 30), 20,
                               lidar_color[obstacle_status], -1)
                    publish_frame = output
                else:
                    placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(placeholder, 'NO CAMERA FRAME',
                                (480, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                    cv2.putText(placeholder, f'CMD: {final_cmd} | {status_text}',
                                (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    if last_error:
                        err = last_error[:120]
                        cv2.putText(placeholder, err,
                                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    publish_frame = placeholder

                publish_telemetry({
                    'running': True,
                    'camera_connected': camera_connected,
                    'stm32_connected': stm32_connected,
                    'stm32_ack_age_ms': float(ack_age_ms),
                    'stm32_encoder_left': int(encoder_left),
                    'stm32_encoder_right': int(encoder_right),
                    'camera_direction': direction,
                    'camera_command': camera_cmd,
                    'lidar_status': obstacle_status,
                    'lidar_connected': lidar_connected,
                    'lidar_error': lidar_error,
                    'lidar_distance_mm': float(obstacle_dist),
                    'final_command': final_cmd,
                    'decision_reason': status_text,
                    'sensor_probes': sensor_probes,
                    'last_error': last_error,
                }, frame=publish_frame, log_message=log_message)

                if SHOW_WINDOW and frame is not None:
                    cv2.imshow("AGV Brain", output)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        stop_reason = 'stop requested from camera window'
                        break

            except KeyboardInterrupt:
                log.info("Stopping by user...")
                stop_reason = 'stop requested by user'
                break
            except Exception as exc:
                import traceback
                tb = traceback.format_exc()
                last_error = f'{type(exc).__name__}: {exc}'
                log.error('[MainLoop]  Unhandled exception — staying alive in degraded mode\n%s', tb)
                try:
                    publish_telemetry({
                        'running': True,
                        'camera_connected': camera_connected,
                        'stm32_connected': stm32_connected,
                        'camera_direction': direction,
                        'camera_command': 'S',
                        'lidar_status': 'UNKNOWN',
                        'lidar_connected': False,
                        'lidar_error': None,
                        'lidar_distance_mm': 0,
                        'final_command': 'S',
                        'decision_reason': f'Main loop error: {last_error}',
                        'sensor_probes': sensor_probes,
                        'last_error': tb,
                    }, log_message=(
                        f"Main loop exception: {last_error}"
                    ), log_category=LOG_CAT_SYSTEM)
                except Exception as publish_exc:
                    log.error('[MainLoop]  Telemetry publish also failed: %s', publish_exc)
                time.sleep(0.5)

    except KeyboardInterrupt:
        log.info("Stopping by user...")
        stop_reason = 'stop requested by user'
    finally:
        # Safety shutdown
        log.warning("Shutting down AGV...")
        if stm:
            send_to_stm32(stm, b'S')
            log.info("   Motors stopped.")
        else:
            log.info("   No STM32 link; no motor command was sent.")

        with lidar_lock:
            lidar_state['running'] = False
        lidar_t.join(timeout=3)
        log.info("   LiDAR thread stopped.")

        with stm32_lock:
            stm32_state['running'] = False
        if stm32_reader_thread is not None:
            stm32_reader_thread.join(timeout=2)
        log.info("   STM32 reader stopped.")

        if cap is not None:
            cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        log.info("   Camera released.")

        if stm:
            stm.close()
            log.info("   STM32 serial closed.")
        log.info("AGV shutdown complete.")
        with lidar_lock:
            lidar_connected = lidar_state['connected']
            lidar_status = lidar_state['status']
            lidar_distance = lidar_state['distance']
            lidar_error = lidar_state['error']
        with stm32_lock:
            encoder_left = stm32_state['encoder_left']
            encoder_right = stm32_state['encoder_right']
            ack_age_ms = (time.monotonic() - stm32_state['last_ack_monotonic']) * 1000.0
        publish_telemetry({
            'running': False,
            'camera_connected': False,
            'stm32_connected': False,
            'stm32_ack_age_ms': float(ack_age_ms),
            'stm32_encoder_left': int(encoder_left),
            'stm32_encoder_right': int(encoder_right),
            'camera_direction': direction if 'direction' in locals() else 'STOPPED',
            'camera_command': camera_cmd if 'camera_cmd' in locals() else 'S',
            'lidar_status': lidar_status,
            'lidar_connected': lidar_connected,
            'lidar_error': lidar_error,
            'lidar_distance_mm': float(lidar_distance),
            'final_command': 'S',
            'sensor_probes': sensor_probes,
            'last_error': last_error,
        }, log_message=[
            (LOG_CAT_CAMERA, 'Camera disconnected (capture closed)'),
            (LOG_CAT_LIDAR,  'LiDAR connected' if lidar_connected else 'LiDAR disconnected'),
            (LOG_CAT_STM32,
             'STM32 serial link disconnected (port closed)' if stm
             else 'STM32 unavailable; camera stopped'),
            (LOG_CAT_SYSTEM, f'Camera stopped ({stop_reason})'),
        ])
        stop_telemetry_writer()


if __name__ == '__main__':
    run_agv()
