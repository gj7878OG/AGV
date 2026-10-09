#!/usr/bin/env python3
"""
LiDAR Collision Monitor — Connected to STM32
=============================================
RPLIDAR A1 on Jetson Nano scans for obstacles and sends
motor commands to STM32 via UART.

Wiring:
  RPLIDAR A1  → Jetson Nano USB (/dev/ttyUSB0)
  Jetson TX   → STM32 PA10 (RX)
  Jetson RX   → STM32 PA9  (TX)
  Jetson GND  → STM32 GND

Commands sent to STM32:
  'S' = Emergency Stop  (obstacle < 1.2m)
  'W' = Slow Forward    (obstacle 1.2m - 2.5m)
  'F' = Full Forward    (path clear)
"""

import time
import math
import serial
from rplidar import RPLidar, RPLidarException

# --- CONFIGURATION ---
LIDAR_PORT = '/dev/ttyUSB0'     # RPLIDAR A1 USB port
STM32_PORT = '/dev/ttyTHS1'     # Jetson Nano UART1 to STM32
BAUD_RATE  = 115200

# Corridor Dimensions (in millimeters)
AGV_HALF_WIDTH = 350            # Protection corridor width (+/- 350mm = 700mm total)
STOP_DISTANCE  = 1200           # Emergency stop threshold (1.2 meters)
SLOW_DISTANCE  = 2500           # Slow-down threshold (2.5 meters)


def connect_stm32():
    """Connect to STM32 via UART. Returns serial object or None."""
    try:
        stm = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)
        print(f"✅ STM32 connected on {STM32_PORT}")
        return stm
    except Exception as e:
        print(f"⚠️  STM32 not connected: {e}")
        print("   Running in MONITOR-ONLY mode (no motor control)")
        return None


def send_command(stm, cmd, obstacle_dist=0):
    """Send command to STM32 and print status."""
    if cmd == "STOP":
        if stm:
            stm.write(b'S')
        print(f"\033[91m[STOP]\033[0m Obstacle at {obstacle_dist/1000:.2f}m → Motors STOPPED")

    elif cmd == "SLOW":
        if stm:
            stm.write(b'W')
        print(f"\033[93m[SLOW]\033[0m Obstacle at {obstacle_dist/1000:.2f}m → Slow forward")

    elif cmd == "CLEAR":
        if stm:
            stm.write(b'F')
        print(f"\033[92m[CLEAR]\033[0m Path clear → Full forward")


def run_lidar():
    print("=" * 50)
    print("  AGV LiDAR Collision Monitor")
    print("  RPLIDAR A1 → Jetson Nano → STM32")
    print("=" * 50)

    # Connect to STM32
    stm = connect_stm32()

    # Connect to LiDAR
    print(f"Connecting to RPLIDAR on {LIDAR_PORT}...")
    lidar = RPLidar(LIDAR_PORT, baudrate=BAUD_RATE, timeout=3)

    try:
        lidar.clean_input()
        lidar.reset()
        time.sleep(1.5)

        info = lidar.get_info()
        print(f"✅ RPLIDAR connected | S/N: {info.get('serialnumber')}")
        print(f"   Stop zone:  < {STOP_DISTANCE/1000:.1f}m")
        print(f"   Slow zone:  {STOP_DISTANCE/1000:.1f}m - {SLOW_DISTANCE/1000:.1f}m")
        print(f"   Corridor:   {AGV_HALF_WIDTH*2/1000:.1f}m wide")
        print("\nMonitoring started... (Ctrl+C to stop)\n")

        last_cmd = None

        for scan in lidar.iter_scans(max_buf_meas=500):
            closest_stop = float('inf')
            closest_slow = float('inf')

            for quality, angle_deg, dist_mm in scan:
                if dist_mm == 0:
                    continue

                # Polar to Cartesian (Forward = Y, Lateral = X)
                theta_rad = math.radians(angle_deg)
                y = dist_mm * math.cos(theta_rad)
                x = dist_mm * math.sin(theta_rad)

                # Only check points inside the AGV's forward path
                if abs(x) <= AGV_HALF_WIDTH and y > 0:
                    if y <= STOP_DISTANCE:
                        closest_stop = min(closest_stop, y)
                    elif y <= SLOW_DISTANCE:
                        closest_slow = min(closest_slow, y)

            # Decide command (only send if changed to avoid UART spam)
            if closest_stop < float('inf'):
                cmd = "STOP"
                dist = closest_stop
            elif closest_slow < float('inf'):
                cmd = "SLOW"
                dist = closest_slow
            else:
                cmd = "CLEAR"
                dist = 0

            if cmd != last_cmd:
                send_command(stm, cmd, dist)
                last_cmd = cmd

    except RPLidarException as e:
        print(f"\n❌ LiDAR error: {e}")
    except KeyboardInterrupt:
        print("\n\nStopping by user...")
    finally:
        # Safety: stop motors before exit
        if stm:
            stm.write(b'S')
            stm.close()
            print("STM32: Motors stopped, serial closed.")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LiDAR safely disconnected.")


if __name__ == '__main__':
    run_lidar()
