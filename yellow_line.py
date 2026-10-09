#!/usr/bin/env python3
"""
Yellow Line Follower — Connected to STM32
==========================================
IMX219 camera on Jetson Nano detects a yellow line and sends
steering commands to STM32 via UART.

Wiring:
  IMX219 Camera → Jetson Nano CSI ribbon
  Jetson TX     → STM32 PA10 (RX)
  Jetson RX     → STM32 PA9  (TX)
  Jetson GND    → STM32 GND

Commands sent to STM32:
  'L' = Turn Left   (yellow line is left of center)
  'R' = Turn Right   (yellow line is right of center)
  'F' = Go Forward   (yellow line is centered)
  'S' = Stop         (no yellow line detected)
"""

import cv2
import numpy as np
import serial
import time

# --- CONFIGURATION ---
STM32_PORT = '/dev/ttyTHS1'     # Jetson Nano UART1 to STM32
BAUD_RATE  = 115200

# Steering thresholds (pixels from center)
STEER_DEADZONE = 80             # Within +/- 80px = centered, go straight

# GStreamer pipeline for IMX219 on Jetson Nano
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


def connect_stm32():
    """Connect to STM32 via UART. Returns serial object or None."""
    try:
        stm = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)
        print(f"✅ STM32 connected on {STM32_PORT}")
        return stm
    except Exception as e:
        print(f"⚠️  STM32 not connected: {e}")
        print("   Running in VISION-ONLY mode (no motor control)")
        return None


def run_line_follower():
    print("=" * 50)
    print("  AGV Yellow Line Follower")
    print("  IMX219 Camera → Jetson Nano → STM32")
    print("=" * 50)

    # Connect to STM32
    stm = connect_stm32()

    # Open camera
    print("Opening IMX219 camera...")
    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("❌ Could not open IMX219 camera")
        if stm:
            stm.close()
        return

    print("✅ Camera opened at 1280x720 @30fps")
    print("Tracking started... (Press Q in window to stop)\n")

    last_cmd = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Could not read frame")
            break

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Yellow HSV range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        output = frame.copy()
        center_x = frame.shape[1] // 2
        cmd = 'S'  # Default: stop if no line

        if contours:
            # Largest yellow contour
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            if area > 500:
                M = cv2.moments(contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Calculate steering error
                    error = cx - center_x

                    if error < -STEER_DEADZONE:
                        direction = "LEFT"
                        cmd = 'L'
                    elif error > STEER_DEADZONE:
                        direction = "RIGHT"
                        cmd = 'R'
                    else:
                        direction = "CENTER"
                        cmd = 'F'

                    # Draw visuals
                    cv2.drawContours(output, [contour], -1, (0, 255, 0), 3)
                    cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.line(output, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)

                    cv2.putText(output, "YELLOW LINE DETECTED",
                                (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(output, f"Direction: {direction} | Cmd: {cmd}",
                                (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(output, f"Error: {error}px",
                                (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            cmd = 'S'
            cv2.putText(output, "NO YELLOW LINE → STOPPED",
                        (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Send command to STM32 (only if changed)
        if cmd != last_cmd:
            if stm:
                stm.write(cmd.encode())
            print(f"  → CMD: {cmd}")
            last_cmd = cmd

        cv2.imshow("AGV Yellow Line", output)
        cv2.imshow("Yellow Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Cleanup
    if stm:
        stm.write(b'S')
        stm.close()
        print("STM32: Motors stopped, serial closed.")
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released.")


if __name__ == '__main__':
    run_line_follower()
