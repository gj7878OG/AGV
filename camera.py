"""Camera abstraction for the AGV.

Supports four backends:
  - argus    : NVIDIA Argus via nvarguscamerasrc (best ISP, JetPack ≤ 6)
  - v4l2     : NVIDIA V4L2 via nvv4l2camerasrc (lower latency)
  - libcamera: libcamera via libcamerasrc (no NVIDIA stack needed)
  - fake     : synthetic yellow line for offline / CI tests

The module exposes a single Camera class that owns the capture and
publishes the latest frame through a thread-safe accessor. The decision
loop never touches OpenCV capture objects directly — it asks the camera
for the next frame.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from config import CAMERA_BACKEND, get_camera_pipeline

log = logging.getLogger('agv.camera')


@dataclass
class CameraReport:
    """Result of probing the camera at startup."""
    backend: str
    pipeline: Optional[str]
    opened: bool
    frame_ok: bool
    error: Optional[str] = None
    latency_ms: float = 0.0


@dataclass
class FrameInfo:
    """A captured frame plus its acquisition timestamp (monotonic)."""
    image: np.ndarray
    monotonic: float
    sequence: int


class FakeCamera:
    """Synthesises a 1280x720 BGR frame with a wobbling yellow line."""

    def __init__(self, width: int = 1280, height: int = 720) -> None:
        self.width = width
        self.height = height
        self._t0 = time.monotonic()
        self._open = True

    def isOpened(self) -> bool: return self._open
    def release(self) -> None: self._open = False

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        if not self._open:
            return False, None
        frame = np.full((self.height, self.width, 3), 80, dtype=np.uint8)  # gray floor
        cx = self.width // 2 + int(120 * np.sin(time.monotonic() - self._t0))
        cv2.line(frame, (cx, 0), (cx, self.height), (0, 255, 255), 12)
        cv2.putText(frame, "FAKE CAMERA — synthetic yellow line",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return True, frame


class Camera:
    """Single-thread capture + thread-safe latest-frame buffer.

    The capture loop runs on a daemon thread so the decision loop is
    never blocked by GStreamer / V4L2 / libcamera latency.
    """

    def __init__(self, backend: Optional[str] = None) -> None:
        self.backend = (backend or CAMERA_BACKEND).lower()
        self.pipeline = get_camera_pipeline(self.backend)
        self._cap: Optional[cv2.VideoCapture] = None
        self._fake: Optional[FakeCamera] = None
        self._lock = threading.Lock()
        self._latest: Optional[FrameInfo] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self._connected = False
        self._last_error: Optional[str] = None

    # ── lifecycle ─────────────────────────────────────────────────────
    def open(self) -> CameraReport:
        t0 = time.monotonic()
        if self.backend == 'fake':
            self._fake = FakeCamera()
            self._connected = True
            self._start_reader()
            return CameraReport(
                backend='fake', pipeline=None, opened=True, frame_ok=True,
                latency_ms=round((time.monotonic() - t0) * 1000, 1),
            )
        if not self.pipeline:
            return CameraReport(self.backend, None, False, False,
                                f'unknown backend {self.backend!r}')
        try:
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        except Exception as e:
            return CameraReport(self.backend, self.pipeline, False, False,
                                f'{type(e).__name__}: {e}')
        if not cap.isOpened():
            cap.release()
            return CameraReport(self.backend, self.pipeline, False, False,
                                'cv2.VideoCapture.isOpened() returned False')
        ret, frame = cap.read()
        if not ret or frame is None:
            cap.release()
            return CameraReport(self.backend, self.pipeline, True, False,
                                'pipeline opened but produced no frames')
        self._cap = cap
        self._connected = True
        self._start_reader()
        return CameraReport(
            backend=self.backend, pipeline=self.pipeline, opened=True, frame_ok=True,
            latency_ms=round((time.monotonic() - t0) * 1000, 1),
        )

    def release(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        if self._cap:
            self._cap.release()
            self._cap = None
        if self._fake:
            self._fake.release()
            self._fake = None
        self._connected = False

    def isOpened(self) -> bool:
        return self._connected

    # ── latest frame ──────────────────────────────────────────────────
    def latest(self) -> Optional[FrameInfo]:
        with self._lock:
            return self._latest

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    # ── internal reader ──────────────────────────────────────────────
    def _start_reader(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._reader, name='camera-reader', daemon=True,
        )
        self._thread.start()

    def _reader(self) -> None:
        backoff = 0.05
        while self._running:
            if self._fake is not None:
                ret, frame = self._fake.read()
            elif self._cap is not None:
                try:
                    ret, frame = self._cap.read()
                except cv2.error as exc:
                    self._last_error = f'cv2.error during read: {exc}'
                    log.error('[Camera] %s', self._last_error)
                    self._connected = False
                    time.sleep(backoff); backoff = min(backoff * 2, 2.0); continue
                except Exception as exc:
                    self._last_error = f'{type(exc).__name__}: {exc}'
                    log.error('[Camera] %s', self._last_error)
                    self._connected = False
                    time.sleep(backoff); backoff = min(backoff * 2, 2.0); continue
            else:
                return
            if not ret or frame is None:
                self._connected = False
                self._last_error = self._last_error or 'read() returned False'
                time.sleep(backoff); backoff = min(backoff * 2, 2.0); continue
            backoff = 0.05
            self._connected = True
            self._seq += 1
            with self._lock:
                self._latest = FrameInfo(image=frame, monotonic=time.monotonic(),
                                         sequence=self._seq)


# Convenience functions kept for backward compatibility with code that
# used the old module pattern.

def detect_yellow_centroid(
    frame: np.ndarray,
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
    min_area: int,
    kernel: tuple[int, int] = (5, 5),
) -> Optional[tuple[int, int, int]]:
    """Return (cx, cy, area) of the largest yellow contour, or None.

    Multi-tier detection. Tries increasingly permissive colour ranges
    so the line stays locked under different lighting and tape colours:

      Tier 1: configured tight yellow range  (H ~20-35, saturated)
      Tier 2: same hue, relaxed S/V          (catches washed-out tape)
      Tier 3: wider warm range               (yellow → chartreuse)
      Tier 4: orange + yellow + yellow-green (H 0-70)
      Tier 5: green range                    (catches "yellow" tape that
                                              the camera has rendered green
                                              under bluish room lighting)

    Tiers 3-5 prefer contours in the lower 2/3 of the frame (the floor)
    unless the contour is very large (it's clearly the line wherever it is).
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    floor_top = h // 3   # below this Y is treated as "floor"
    k = np.ones(kernel, np.uint8)

    def _scan(low_h: int, high_h: int, min_s: int, min_v: int,
              min_area_override: Optional[int] = None
    ) -> Optional[tuple[int, int, int]]:
        mask = cv2.inRange(
            hsv,
            np.array((low_h, min_s, min_v)),
            np.array((high_h, 255, 255)),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area = int(cv2.contourArea(contour))
        threshold = min_area_override if min_area_override is not None else min_area
        if area < threshold:
            return None
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        return cx, cy, area

    # Tier 1: configured range, anywhere in the frame
    result = _scan(lower[0], upper[0], lower[1], lower[2])
    if result is not None:
        return result

    # Tier 2: same hue, relaxed saturation/value
    result = _scan(lower[0], upper[0], 50, 50)
    if result is not None:
        return result

    # Tier 3: broader warm range — yellow-green / chartreuse
    result = _scan(10, 45, 50, 50)
    if result is not None:
        cx, cy, area = result
        if cy >= floor_top or area >= 4 * min_area:
            return result

    # Tier 4: orange → yellow → chartreuse (H 0-70, slightly stricter)
    result = _scan(0, 70, 60, 60)
    if result is not None:
        cx, cy, area = result
        if cy >= floor_top or area >= 8 * min_area:
            return result

    # Tier 5: GREEN range — last resort. Under very blue room lighting,
    # a yellow tape can render as green-cyan. Catches the line even
    # when the camera's white balance is fighting us.
    # Require a high saturation so we don't pick up a green floor.
    result = _scan(35, 90, 100, 80)
    if result is not None:
        cx, cy, area = result
        if cy >= floor_top or area >= 6 * min_area:
            return result

    return None