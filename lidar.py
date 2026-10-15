"""LiDAR abstraction for the AGV.

Wraps an RPLIDAR A1 in a background thread and publishes the latest
processed obstacle state (CLEAR / SLOW / STOP + mm) through a thread-safe
accessor.

The class keeps the same API surface as the original lidar_state dict so
older call sites still compile, but adds:
  - dataclass state (typed)
  - explicit shutdown via .close()
  - automatic reconnection (every RECONNECT_S) if the sensor drops
  - context-manager support
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from rplidar import RPLidar, RPLidarException

from config import (
    LIDAR_PORT, BAUD_RATE, AGV_HALF_WIDTH, STOP_DISTANCE, SLOW_DISTANCE,
)

log = logging.getLogger('agv.lidar')

# Distance bucket width used for log "telemetry" deduplication.
from config import LIDAR_BUCKET_MM  # noqa: E402


@dataclass
class LidarState:
    """Latest known obstacle state from the scanner."""
    status: str = 'CLEAR'        # 'CLEAR' | 'SLOW' | 'STOP' | 'UNKNOWN'
    distance: float = 0.0        # mm, closest obstacle in the corridor
    connected: bool = False
    error: Optional[str] = None
    running: bool = True


@dataclass
class LidarReport:
    """Result of probing the LiDAR at startup."""
    port: str
    opened: bool
    serial: Optional[str] = None
    error: Optional[str] = None
    latency_ms: float = 0.0


class Lidar:
    """Background scanner; thread-safe latest-state accessor."""

    RECONNECT_S = 2.0             # retry cadence when scanner drops

    def __init__(self, port: Optional[str] = None, baud: Optional[int] = None) -> None:
        self.port = port or LIDAR_PORT
        self.baud = baud or BAUD_RATE
        self._state = LidarState()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    # ── context manager ──────────────────────────────────────────────
    def __enter__(self) -> 'Lidar':
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    # ── lifecycle ─────────────────────────────────────────────────────
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name='lidar-reader', daemon=True,
            )
            self._thread.start()

    def close(self) -> None:
        with self._lock:
            self._state.running = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None

    # ── state access ─────────────────────────────────────────────────
    def snapshot(self) -> LidarState:
        with self._lock:
            return LidarState(
                status=self._state.status,
                distance=self._state.distance,
                connected=self._state.connected,
                error=self._state.error,
                running=self._state.running,
            )

    def set_running(self, value: bool) -> None:
        with self._lock:
            self._state.running = value

    # ── compatibility shims ──────────────────────────────────────────
    @property
    def status(self) -> str:
        with self._lock: return self._state.status

    @property
    def distance(self) -> float:
        with self._lock: return self._state.distance

    @property
    def connected(self) -> bool:
        with self._lock: return self._state.connected

    @property
    def error(self) -> Optional[str]:
        with self._lock: return self._state.error

    @property
    def running(self) -> bool:
        with self._lock: return self._state.running

    # ── probe (synchronous; no background thread) ────────────────────
    def probe(self) -> LidarReport:
        """One-shot connect to read serial number; closes immediately."""
        t0 = time.monotonic()
        lidar = None
        try:
            lidar = RPLidar(self.port, baudrate=self.baud, timeout=3)
            info = lidar.get_info()
            return LidarReport(
                port=self.port, opened=True,
                serial=info.get('serialnumber'),
                latency_ms=round((time.monotonic() - t0) * 1000, 1),
            )
        except RPLidarException as e:
            return LidarReport(self.port, False,
                               error=f'RPLidarException: {e}',
                               latency_ms=round((time.monotonic() - t0) * 1000, 1))
        except Exception as e:
            return LidarReport(self.port, False,
                               error=f'{type(e).__name__}: {e}',
                               latency_ms=round((time.monotonic() - t0) * 1000, 1))
        finally:
            if lidar:
                try:
                    lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                except Exception:
                    pass

    # ── background loop ──────────────────────────────────────────────
    def _run(self) -> None:
        log.info('[LiDAR] Connecting to RPLIDAR on %s...', self.port)
        while True:
            with self._lock:
                if not self._state.running:
                    break
            lidar = None
            try:
                lidar = RPLidar(self.port, baudrate=self.baud, timeout=3)
                lidar.clean_input()
                lidar.reset()
                time.sleep(1.5)
                info = lidar.get_info()
                log.info('[LiDAR]   Connected | S/N: %s', info.get('serialnumber'))
                log.info('[LiDAR]     Stop: <%.1fm | Slow: <%.1fm | Width: %.1fm',
                         STOP_DISTANCE / 1000, SLOW_DISTANCE / 1000,
                         AGV_HALF_WIDTH * 2 / 1000)
                with self._lock:
                    self._state.connected = True
                    self._state.error = None

                for scan in lidar.iter_scans(max_buf_meas=500):
                    with self._lock:
                        if not self._state.running:
                            break
                    closest_stop = float('inf')
                    closest_slow = float('inf')
                    for _quality, angle_deg, dist_mm in scan:
                        if dist_mm == 0:
                            continue
                        theta = math.radians(angle_deg)
                        y = dist_mm * math.cos(theta)
                        x = dist_mm * math.sin(theta)
                        if abs(x) <= AGV_HALF_WIDTH and y > 0:
                            if y <= STOP_DISTANCE:
                                closest_stop = min(closest_stop, y)
                            elif y <= SLOW_DISTANCE:
                                closest_slow = min(closest_slow, y)
                    with self._lock:
                        if closest_stop < float('inf'):
                            self._state.status = 'STOP'
                            self._state.distance = closest_stop
                        elif closest_slow < float('inf'):
                            self._state.status = 'SLOW'
                            self._state.distance = closest_slow
                        else:
                            self._state.status = 'CLEAR'
                            self._state.distance = 0.0

            except RPLidarException as exc:
                log.error('[LiDAR]   Error: %s', exc)
                with self._lock:
                    self._state.connected = False
                    self._state.error = str(exc)
            except Exception as exc:
                log.error('[LiDAR]   Unexpected: %s', exc)
                with self._lock:
                    self._state.connected = False
                    self._state.error = f'{type(exc).__name__}: {exc}'
            finally:
                if lidar:
                    try:
                        lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                    except Exception:
                        pass
                with self._lock:
                    self._state.connected = False
                # Bail out if shutting down; otherwise back off and retry.
                with self._lock:
                    still_running = self._state.running
                if not still_running:
                    break
                log.info('[LiDAR] Reconnecting in %.1fs...', self.RECONNECT_S)
                time.sleep(self.RECONNECT_S)
        log.info('[LiDAR] Reader thread exited.')


def distance_bucket(mm: float) -> int:
    """Bucket distance in LIDAR_BUCKET_MM bands for log deduplication."""
    return int(mm // LIDAR_BUCKET_MM) if mm else 0