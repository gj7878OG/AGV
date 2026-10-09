#!/usr/bin/env python3
"""
AGV Main Brain — Unified Control System
========================================
Runs BOTH LiDAR obstacle avoidance AND camera line following
on Jetson Nano, with a single UART connection to STM32.

Priority Logic:
  1. LiDAR STOP  → Emergency stop  (highest priority)
  2. LiDAR SLOW  → Slow forward    (overrides camera steering)
  3. Camera LEFT  → Turn left       (line following)
  4. Camera RIGHT → Turn right      (line following)
  5. Camera CENTER → Full forward   (line following)
  6. No line      → Stop            (safety fallback)

Wiring:
  RPLIDAR A1    → Jetson Nano USB (/dev/ttyUSB0)
  IMX219 Camera → Jetson Nano CSI ribbon
  Jetson Pin 8  (TX) → STM32 PA10 (RX)
  Jetson Pin 10 (RX) → STM32 PA9  (TX)
  Jetson Pin 6  (GND) → STM32 GND

Dependencies (install on Jetson Nano):
  pip3 install pyserial rplidar-roboticia

Usage:
  python3 agv_main.py
"""

import cv2
import numpy as np
import serial
import time
import math
import threading
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
    'running': True,         # Set False to kill the thread
}
lidar_lock = threading.Lock()


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
    """Send a single command byte to STM32."""
    if stm:
        try:
            stm.write(cmd_byte)
        except serial.SerialException:
            pass


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
    except Exception as e:
        print(f"[LiDAR] ❌ Unexpected error: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        print("[LiDAR] Disconnected.")


# ─────────────────────────────────────────────
#  CAMERA + DECISION LOOP (main thread)
# ─────────────────────────────────────────────

def run_agv():
    global lidar_state

    print("=" * 55)
    print("  AGV MAIN BRAIN — Unified Control System")
    print("  LiDAR + Camera → Jetson Nano → STM32")
    print("=" * 55)
    print()

    # 1. Connect to STM32
    stm = connect_stm32()
    if not stm:
        print("\n⛔ Cannot proceed without STM32. Check wiring.")
        return

    # Safety: stop motors
    send_to_stm32(stm, b'S')

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
        return

    print("[Camera] ✅ IMX219 opened at 1280x720 @30fps")
    print("\n🚀 AGV RUNNING — Press Q in camera window to stop\n")

    last_cmd = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # ── Read LiDAR state ──
            with lidar_lock:
                obstacle_status = lidar_state['status']
                obstacle_dist = lidar_state['distance']

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

            # ─────────────────────────────────
            #  PRIORITY DECISION ENGINE
            # ─────────────────────────────────
            #  LiDAR STOP  → 'S' (highest)
            #  LiDAR SLOW  → 'W' (override camera)
            #  Camera cmd  → 'L'/'R'/'F'/'S'
            # ─────────────────────────────────

            if obstacle_status == 'STOP':
                final_cmd = 'S'
                status_text = f"EMERGENCY STOP | Obstacle {obstacle_dist/1000:.2f}m"
                status_color = (0, 0, 255)  # Red
            elif obstacle_status == 'SLOW':
                final_cmd = 'W'
                status_text = f"SLOW | Obstacle {obstacle_dist/1000:.2f}m"
                status_color = (0, 255, 255)  # Yellow
            else:
                final_cmd = camera_cmd
                status_text = f"Line: {direction}"
                status_color = (0, 255, 0)  # Green

            # Send command to STM32 (only on change)
            if final_cmd != last_cmd:
                send_to_stm32(stm, final_cmd.encode())
                print(f"  → STM32: '{final_cmd}' | LiDAR: {obstacle_status} | Camera: {direction}")
                last_cmd = final_cmd

            # ── Draw HUD on camera feed ──
            # Status bar
            cv2.rectangle(output, (0, 0), (frame.shape[1], 60), (0, 0, 0), -1)
            cv2.putText(output, f"CMD: {final_cmd} | {status_text}",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

            # LiDAR indicator
            lidar_color = {'CLEAR': (0, 255, 0), 'SLOW': (0, 255, 255), 'STOP': (0, 0, 255)}
            cv2.circle(output, (frame.shape[1] - 40, 30), 20, lidar_color[obstacle_status], -1)

            cv2.imshow("AGV Brain", output)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n\nStopping by user...")
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
        cv2.destroyAllWindows()
        print("   Camera released.")

        stm.close()
        print("   STM32 serial closed.")
        print("\n✅ AGV shutdown complete.")


if __name__ == '__main__':
    run_agv()
