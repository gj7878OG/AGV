#!/usr/bin/env python3
"""Standalone yellow-line follower.

Same behaviour as before, but uses the new modular split:

  camera  → Camera (any backend)
  stm32   → STM32 (ACK/encoder + commands)
  decision→ pure line-following verdict
"""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

from camera import Camera, detect_yellow_centroid
from config import (CAMERA_BACKEND, MIN_CONTOUR_AREA, MORPHOLOGY_KERNEL,
                   SHOW_WINDOW, STEER_DEADZONE, YELLOW_HSV_LOWER,
                   YELLOW_HSV_UPPER)
from decision import Direction
from stm32 import STM32


def draw_overlay(frame: np.ndarray, direction: Direction,
                 centroid: tuple[int, int] | None, cmd: str, error: int) -> None:
    if centroid is not None:
        cx, cy = centroid
        cv2.drawContours(frame, [cv2.convexHull(
            np.array([[cx - 30, cy - 30], [cx + 30, cy - 30],
                       [cx + 30, cy + 30], [cx - 30, cy + 30]],
                      dtype=np.int32))], -1, (0, 255, 0), 3)
        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
        cv2.line(frame, (frame.shape[1] // 2, 0),
                 (frame.shape[1] // 2, frame.shape[0]), (255, 0, 0), 2)

    cv2.putText(frame, 'YELLOW LINE DETECTED' if centroid is not None
                else 'NO YELLOW LINE → STOPPED',
                (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 255, 0) if centroid is not None else (0, 0, 255), 2)
    cv2.putText(frame, f'Direction: {direction.value} | Cmd: {cmd}',
                (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(frame, f'Error: {error}px',
                (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(levelname)s] %(message)s')
    log = logging.getLogger('agv.line')

    print('=' * 50)
    print('  AGV Yellow Line Follower')
    print('  IMX219 Camera → Jetson Nano → STM32')
    print('=' * 50)

    stm = STM32()
    report = stm.connect()
    if not report.opened:
        print('⚠ STM32 unavailable — vision-only mode.')

    cam = Camera()
    report = cam.open()
    if not report.opened:
        print('❌ Could not open camera')
        stm.close()
        return
    print('✅ Camera opened')

    last_cmd = None
    error = 0
    direction = Direction.UNAVAILABLE
    cmd = 'S'
    centroid = None

    try:
        while True:
            info = cam.latest()
            frame = info.image if info else None
            if frame is None:
                time.sleep(0.02)
                continue

            detected = detect_yellow_centroid(
                frame,
                lower=YELLOW_HSV_LOWER, upper=YELLOW_HSV_UPPER,
                min_area=MIN_CONTOUR_AREA, kernel=MORPHOLOGY_KERNEL,
            )
            if detected is None:
                direction = Direction.LOST
                cmd = 'S'
                centroid = None
            else:
                cx, cy, _area = detected
                centroid = (cx, cy)
                center_x = frame.shape[1] // 2
                error = cx - center_x
                if error < -STEER_DEADZONE:
                    direction = Direction.LEFT; cmd = 'L'
                elif error > STEER_DEADZONE:
                    direction = Direction.RIGHT; cmd = 'R'
                else:
                    direction = Direction.CENTER; cmd = 'F'

            if cmd != last_cmd:
                stm.send(cmd)
                print(f'  → CMD: {cmd}')
                last_cmd = cmd

            if SHOW_WINDOW:
                output = frame.copy()
                draw_overlay(output, direction, centroid, cmd, error)
                cv2.imshow('AGV Yellow Line', output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if stm.isOpen():
            stm.send('S')
            stm.close()
            print('STM32: Motors stopped, serial closed.')
        cam.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        print('Camera released.')


if __name__ == '__main__':
    main()