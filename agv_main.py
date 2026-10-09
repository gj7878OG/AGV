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
import os
import tempfile
from collections import deque
from rplidar import RPLidar, RPLidarException

# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

# Serial ports
LIDAR_PORT = '/dev/ttyUSB0'     # RPLIDAR A1 USB
STM32_PORT = '/dev/ttyTHS1'     # Jetson UART1 to STM32
BAUD_RATE  = 115200

# LiDAR collision zones (millimeters)
AGV_HALF_WIDTH = 350            # Corridor half-width (700mm total)
STOP_DISTANCE  = 1200           # Emergency stop zone (< 1.2m)
SLOW_DISTANCE  = 2500           # Slow-down zone (1.2m - 2.5m)

# Camera steering
STEER_DEADZONE = 80             # Pixels from center before steering
TELEMETRY_DIR = os.environ.get('AGV_DASHBOARD_DIR', '/tmp/agv-dashboard')
SHOW_WINDOW = os.environ.get(
    'AGV_SHOW_WINDOW', '1' if os.environ.get('DISPLAY') else '0'
).lower() in ('1', 'true', 'yes', 'on')
LOG_LIMIT = 500

# GStreamer pipeline for IMX219
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


def publish_telemetry(state, frame=None, log_message=None):
    """Queue read-only state and frame; disk I/O stays off the control loop."""
    global telemetry_log_id, telemetry_pending

    with telemetry_lock:
        messages = log_message if isinstance(log_message, (list, tuple)) else [log_message]
        for message in messages:
            if message:
                telemetry_log_id += 1
                telemetry_logs.append({
                    'id': telemetry_log_id,
                    'timestamp': time.time(),
                    'message': message,
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
            print(f"[Dashboard] Telemetry publish failed: {exc}")
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
            telemetry_logs.append(entry)
            telemetry_log_id = max(telemetry_log_id, int(entry.get('id', 0)))
    except (OSError, ValueError, TypeError):
        pass


# ─────────────────────────────────────────────
#  STM32 SERIAL
# ─────────────────────────────────────────────

def connect_stm32():
    """Connect to STM32 via UART."""
    try:
        stm = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)
        print(f"✅ STM32 connected on {STM32_PORT}")
        return stm
    except Exception as e:
        print(f"❌ Cannot connect STM32 on {STM32_PORT}: {e}")
        print("   AGV will not move without STM32 connection.")
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

    print(f"[LiDAR] Connecting to RPLIDAR on {LIDAR_PORT}...")
    lidar = None

    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_RATE, timeout=3)
        lidar.clean_input()
        lidar.reset()
        time.sleep(1.5)

        info = lidar.get_info()
        print(f"[LiDAR] ✅ Connected | S/N: {info.get('serialnumber')}")
        print(f"[LiDAR]    Stop: <{STOP_DISTANCE/1000:.1f}m | Slow: <{SLOW_DISTANCE/1000:.1f}m | Width: {AGV_HALF_WIDTH*2/1000:.1f}m")
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
        print(f"[LiDAR] ❌ Error: {e}")
        with lidar_lock:
            lidar_state['connected'] = False
            lidar_state['error'] = str(e)
    except Exception as e:
        print(f"[LiDAR] ❌ Unexpected error: {e}")
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
        print("[LiDAR] Disconnected.")


# ─────────────────────────────────────────────
#  CAMERA + DECISION LOOP (main thread)
# ─────────────────────────────────────────────

def run_agv():
    global lidar_state

    restore_telemetry_logs()
    start_telemetry_writer()

    print("=" * 55)
    print("  AGV MAIN BRAIN — Unified Control System")
    print("  LiDAR + Camera → Jetson Nano → STM32")
    print("=" * 55)
    print()
    with lidar_lock:
        lidar_state['running'] = True
        lidar_state['status'] = 'CLEAR'
        lidar_state['distance'] = 0
        lidar_state['connected'] = False
        lidar_state['error'] = None
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
    }, log_message='AGV control process starting')

    # 1. Connect to STM32
    stm = connect_stm32()
    if not stm:
        print("\n⛔ Cannot proceed without STM32. Check wiring.")
        publish_telemetry({
            'running': False,
            'camera_connected': False,
            'stm32_connected': False,
            'camera_direction': 'UNAVAILABLE',
            'camera_command': 'S',
            'lidar_status': 'CLEAR',
            'lidar_connected': False,
            'lidar_error': None,
            'lidar_distance_mm': 0,
            'final_command': 'S',
        }, log_message='STM32 unavailable; AGV did not start')
        stop_telemetry_writer()
        return

    # Safety: stop motors
    stm32_connected = send_to_stm32(stm, b'S')
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
    }, log_message=(
        'STM32 connected; initial stop command sent' if stm32_connected
        else 'STM32 command write failed during startup'
    ))

    # 2. Start LiDAR in background thread
    lidar_t = threading.Thread(target=lidar_thread_func, daemon=True)
    lidar_t.start()
    time.sleep(2)  # Let LiDAR initialize

    # 3. Open camera
    print("\n[Camera] Opening IMX219...")
    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[Camera] ❌ Could not open IMX219")
        send_to_stm32(stm, b'S')
        stm.close()
        with lidar_lock:
            lidar_state['running'] = False
        lidar_t.join(timeout=3)
        with lidar_lock:
            lidar_connected = lidar_state['connected']
            lidar_status = lidar_state['status']
            lidar_distance = lidar_state['distance']
            lidar_error = lidar_state['error']
        publish_telemetry({
            'running': False,
            'camera_connected': False,
            'stm32_connected': False,
            'camera_direction': 'UNAVAILABLE',
            'camera_command': 'S',
            'lidar_status': lidar_status,
            'lidar_connected': lidar_connected,
            'lidar_error': lidar_error,
            'lidar_distance_mm': float(lidar_distance),
            'final_command': 'S',
        }, log_message='Camera unavailable; AGV did not start')
        stop_telemetry_writer()
        return

    print("[Camera] ✅ IMX219 opened at 1280x720 @30fps")
    camera_connected = True
    if SHOW_WINDOW:
        print("\n🚀 AGV RUNNING — Press Q in camera window to stop\n")
    else:
        print("\n🚀 AGV RUNNING headlessly — Press Ctrl+C to stop\n")

    last_cmd = None
    last_camera_state = None
    last_camera_connection = None
    last_lidar_state = None
    last_lidar_connection = None
    last_stm32_connection = None
    last_final_cmd = None
    stop_reason = 'camera stream ended'

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                camera_connected = False
                stop_reason = 'camera frame read failed'
                print('[Camera] ❌ Frame read failed; stopping AGV')
                break

            # ── Read LiDAR state ──
            with lidar_lock:
                obstacle_status = lidar_state['status']
                obstacle_dist = lidar_state['distance']
                lidar_connected = lidar_state['connected']
                lidar_error = lidar_state['error']

            # ── Camera: detect yellow line ──
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([35, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            output = frame.copy()
            center_x = frame.shape[1] // 2
            camera_cmd = 'S'
            direction = "NO LINE"

            if contours:
                contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(contour)

                if area > 500:
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

            # ─────────────────────────────────────────────
            #  DECISION ENGINE
            #  PRIMARY:  Camera (OpenCV yellow line following)
            #  SAFETY:   LiDAR (emergency stop only)
            # ─────────────────────────────────────────────
            #  Camera decides direction: L / R / F / S
            #  LiDAR only overrides if obstacle < 1.2m
            # ─────────────────────────────────────────────

            # Camera is ALWAYS the primary decision maker
            final_cmd = camera_cmd
            status_text = f"Line: {direction}"
            status_color = (0, 255, 0)  # Green

            # LiDAR safety override — ONLY when about to crash
            if obstacle_status == 'STOP':
                # Emergency: obstacle too close, override everything
                final_cmd = 'S'
                status_text = f"⚠ SAFETY STOP | Obstacle {obstacle_dist/1000:.2f}m"
                status_color = (0, 0, 255)  # Red
            elif obstacle_status == 'SLOW' and camera_cmd == 'F':
                # Obstacle ahead but not critical — camera still steers, just slower
                final_cmd = 'W'
                status_text = f"Line: {direction} | Obstacle {obstacle_dist/1000:.2f}m (slowing)"
                status_color = (0, 255, 255)  # Yellow

            # Send command to STM32 (only on change)
            if final_cmd != last_cmd:
                stm32_connected = send_to_stm32(stm, final_cmd.encode())
                log_message = (
                    f"STM32: {final_cmd} | Camera: {direction} | "
                    f"LiDAR: {obstacle_status} ({obstacle_dist:.0f} mm)"
                )
                print(f"  → {log_message}")
                last_cmd = final_cmd
            else:
                log_message = None

            # Record meaningful transitions, not every camera frame or raw
            # LiDAR distance fluctuation. Distance changes are grouped in 250mm bands.
            events = []
            camera_state = (direction, camera_cmd)
            lidar_bucket = int(obstacle_dist // 250) if obstacle_dist else 0
            current_lidar_state = (obstacle_status, lidar_bucket)
            if last_camera_state is not None and camera_state != last_camera_state:
                events.append(
                    f"Camera: {last_camera_state[0]} ({last_camera_state[1]}) → "
                    f"{direction} ({camera_cmd})"
                )
            elif last_camera_state is None:
                events.append(f"Camera state: {direction} ({camera_cmd})")
            if last_camera_connection is None:
                events.append('Camera connected' if camera_connected else 'Camera disconnected')
            elif camera_connected != last_camera_connection:
                events.append('Camera connected' if camera_connected else 'Camera disconnected')
            if last_stm32_connection is None:
                events.append('STM32 connected' if stm32_connected else 'STM32 disconnected')
            elif stm32_connected != last_stm32_connection:
                events.append('STM32 connected' if stm32_connected else 'STM32 disconnected')
            if last_lidar_connection is not None and lidar_connected != last_lidar_connection:
                events.append(
                    'LiDAR connected' if lidar_connected else
                    f"LiDAR disconnected{': ' + lidar_error if lidar_error else ''}"
                )
            elif last_lidar_connection is None:
                events.append(
                    'LiDAR connected' if lidar_connected else
                    f"LiDAR unavailable{': ' + lidar_error if lidar_error else ' (connecting or not detected)'}"
                )
            if last_lidar_state is not None and current_lidar_state != last_lidar_state:
                old_distance = (
                    f"{last_lidar_state[1] * 250}–{(last_lidar_state[1] + 1) * 250} mm"
                    if last_lidar_state[1] else 'no obstacle'
                )
                new_distance = f"{obstacle_dist:.0f} mm" if obstacle_dist else 'no obstacle'
                events.append(
                    f"LiDAR: {last_lidar_state[0]} {old_distance} → "
                    f"{obstacle_status} {new_distance}"
                )
            elif last_lidar_state is None:
                distance_text = f" at {obstacle_dist:.0f} mm" if obstacle_dist else ''
                events.append(f"LiDAR state: {obstacle_status}{distance_text}")
            if last_final_cmd is not None and final_cmd != last_final_cmd:
                events.append(f"Final command: {last_final_cmd} → {final_cmd}")
            elif last_final_cmd is None:
                events.append(f"Final command: {final_cmd}")
            if events:
                log_message = events
            last_camera_state = camera_state
            last_camera_connection = camera_connected
            last_lidar_state = current_lidar_state
            last_lidar_connection = lidar_connected
            last_stm32_connection = stm32_connected
            last_final_cmd = final_cmd

            # ── Draw HUD on camera feed ──
            # Status bar
            cv2.rectangle(output, (0, 0), (frame.shape[1], 60), (0, 0, 0), -1)
            cv2.putText(output, f"CMD: {final_cmd} | {status_text}",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

            # LiDAR indicator
            lidar_color = {'CLEAR': (0, 255, 0), 'SLOW': (0, 255, 255), 'STOP': (0, 0, 255)}
            cv2.circle(output, (frame.shape[1] - 40, 30), 20, lidar_color[obstacle_status], -1)

            publish_telemetry({
                'running': True,
                'camera_connected': camera_connected,
                'stm32_connected': stm32_connected,
                'camera_direction': direction,
                'camera_command': camera_cmd,
                'lidar_status': obstacle_status,
                'lidar_connected': lidar_connected,
                'lidar_error': lidar_error,
                'lidar_distance_mm': float(obstacle_dist),
                'final_command': final_cmd,
                'decision_reason': status_text,
            }, frame=output, log_message=log_message)

            if SHOW_WINDOW:
                cv2.imshow("AGV Brain", output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_reason = 'stop requested from camera window'
                    break

    except KeyboardInterrupt:
        print("\n\nStopping by user...")
        stop_reason = 'stop requested by user'
    finally:
        # Safety shutdown
        print("\n🛑 Shutting down AGV...")
        send_to_stm32(stm, b'S')
        print("   Motors stopped.")

        with lidar_lock:
            lidar_state['running'] = False
        lidar_t.join(timeout=3)
        print("   LiDAR thread stopped.")

        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        print("   Camera released.")

        stm.close()
        print("   STM32 serial closed.")
        print("\n✅ AGV shutdown complete.")
        with lidar_lock:
            lidar_connected = lidar_state['connected']
            lidar_status = lidar_state['status']
            lidar_distance = lidar_state['distance']
            lidar_error = lidar_state['error']
        publish_telemetry({
            'running': False,
            'camera_connected': False,
            'stm32_connected': False,
            'camera_direction': direction if 'direction' in locals() else 'STOPPED',
            'camera_command': camera_cmd if 'camera_cmd' in locals() else 'S',
            'lidar_status': lidar_status,
            'lidar_connected': lidar_connected,
            'lidar_error': lidar_error,
            'lidar_distance_mm': float(lidar_distance),
            'final_command': 'S',
        }, log_message=[
            'Camera disconnected (capture closed)',
            'LiDAR connected' if lidar_connected else 'LiDAR disconnected',
            'STM32 serial link disconnected (port closed)',
            f'AGV stopped ({stop_reason}); STM32 stop command sent',
        ])
        stop_telemetry_writer()


if __name__ == '__main__':
    run_agv()
