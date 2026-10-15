"""Line-following decision engine for the AGV.

The decision engine combines:
  - Camera yellow-line detection (PRIMARY)
  - LiDAR obstacle state (SAFETY override only)
  - STM32 link health (force-stop on stale ACK)

It is pure logic — no I/O, no threads. Feed it inputs, get a decision.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import numpy as np

from camera import detect_yellow_centroid
from config import (
    STEER_DEADZONE, MIN_CONTOUR_AREA, MORPHOLOGY_KERNEL,
    YELLOW_HSV_LOWER, YELLOW_HSV_UPPER,
    STOP_DISTANCE, SLOW_DISTANCE,
    STM32_ACK_TIMEOUT_MS,
)


class Direction(str, Enum):
    """Camera-side line-following verdict."""
    LEFT = 'LEFT'
    CENTER = 'CENTER'
    RIGHT = 'RIGHT'
    LOST = 'LOST'            # no yellow line visible
    UNAVAILABLE = 'UNAVAILABLE'  # no camera frame


# Camera → STM32 command byte.
DIR_TO_CMD = {
    Direction.LEFT:  'L',
    Direction.CENTER: 'F',
    Direction.RIGHT: 'R',
    Direction.LOST:  'S',
    Direction.UNAVAILABLE: 'S',
}


@dataclass(frozen=True)
class LineObservation:
    direction: Direction
    centroid_xy: Optional[Tuple[int, int]] = None  # (cx, cy) in pixels
    area: int = 0


@dataclass(frozen=True)
class LidarObservation:
    status: str          # 'CLEAR' | 'SLOW' | 'STOP' | 'UNKNOWN'
    distance_mm: float
    connected: bool


@dataclass(frozen=True)
class LinkObservation:
    stm32_alive: bool
    ack_age_ms: float


@dataclass(frozen=True)
class Decision:
    """Result of one decision-engine tick."""
    camera_cmd: str          # what the camera wants ('L', 'F', 'R', 'S')
    final_cmd: str           # what to send to the STM32
    speed: Optional[str]     # '2' (slow) or '5' (fast); None when not F
    status_text: str         # human-readable reason
    status_color: Tuple[int, int, int]   # BGR for the HUD overlay
    slow_requested: bool


def observe_frame(frame: Optional[np.ndarray]) -> LineObservation:
    """Camera input → line-following verdict."""
    if frame is None:
        return LineObservation(Direction.UNAVAILABLE)
    centroid = detect_yellow_centroid(
        frame,
        lower=YELLOW_HSV_LOWER,
        upper=YELLOW_HSV_UPPER,
        min_area=MIN_CONTOUR_AREA,
        kernel=MORPHOLOGY_KERNEL,
    )
    if centroid is None:
        return LineObservation(Direction.LOST)
    cx, cy, area = centroid
    width = frame.shape[1]
    center_x = width // 2
    error = cx - center_x
    if error < -STEER_DEADZONE:
        d = Direction.LEFT
    elif error > STEER_DEADZONE:
        d = Direction.RIGHT
    else:
        d = Direction.CENTER
    return LineObservation(direction=d, centroid_xy=(cx, cy), area=area)


def decide(
    line: LineObservation,
    lidar: LidarObservation,
    link: LinkObservation,
) -> Decision:
    """Combine all observations into a final motor command."""
    camera_cmd = DIR_TO_CMD[line.direction]

    # Default: trust the camera.
    final_cmd = camera_cmd
    speed: Optional[str] = None
    slow_requested = False
    status_text = f'Line: {line.direction.value}'
    status_color: Tuple[int, int, int] = (0, 255, 0)  # green

    # 1) LiDAR emergency-stop has absolute priority.
    if lidar.status == 'STOP':
        final_cmd = 'S'
        speed = None
        status_text = f'⚠ SAFETY STOP | Obstacle {lidar.distance_mm / 1000:.2f}m'
        status_color = (0, 0, 255)
    # 2) LiDAR slow-zone downshifts forward motion only.
    elif lidar.status == 'SLOW' and camera_cmd == 'F':
        final_cmd = 'F'
        speed = '2'                 # 40 %
        slow_requested = True
        status_text = (f'Line: {line.direction.value} | '
                       f'Obstacle {lidar.distance_mm / 1000:.2f}m (slowing)')
        status_color = (0, 255, 255)
    else:
        speed = '5' if camera_cmd == 'F' else None  # 100 % when moving fast

    # 3) If any required input is missing, hold at S.
    if line.direction is Direction.UNAVAILABLE or not lidar.connected:
        final_cmd = 'S'
        speed = None
        slow_requested = False
        if line.direction is Direction.UNAVAILABLE and not lidar.connected:
            status_text = 'CAMERA ONLY | LiDAR unavailable'
        elif not lidar.connected:
            status_text = 'CAMERA ONLY | LiDAR unavailable'
        else:
            status_text = 'NO CAMERA | LiDAR running'
        status_color = (0, 0, 255)

    # 4) STM32 watchdog trips on lost ACK → instant STOP.
    if not link.stm32_alive:
        final_cmd = 'S'
        speed = None
        slow_requested = False
        status_text = (f'STM32 WATCHDOG ({link.ack_age_ms:.0f} ms since ACK)')
        status_color = (0, 0, 255)

    return Decision(
        camera_cmd=camera_cmd,
        final_cmd=final_cmd,
        speed=speed,
        status_text=status_text,
        status_color=status_color,
        slow_requested=slow_requested,
    )