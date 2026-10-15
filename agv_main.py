#!/usr/bin/env python3
"""AGV orchestrator — thin controller over the new module split.

  camera    → Camera class (argus / v4l2 / libcamera / fake)
  lidar     → Lidar class (RPLIDAR A1, thread-safe snapshots)
  stm32     → STM32 class (UART commands, ACK + encoder telemetry)
  decision  → pure-function combine(camera, lidar, link) → Decision
  telemetry   → TelemetryPublisher (atomic status.json + frame.jpg)

This file does only one thing: tie those pieces together in the main loop
and decide when to publish a state transition. All I/O, threading and
serial-protocol details live in the modules.
"""

from __future__ import annotations

import logging
import os
import signal
import threading
import time
import traceback
from typing import Optional

import numpy as np

from camera import Camera, CameraReport
from config import (
    AGV_HALF_WIDTH, BAUD_RATE, LIDAR_PORT, LOG_FILE, SHOW_WINDOW,
    STOP_DISTANCE, SLOW_DISTANCE, STM32_PORT,
)
from decision import (
    Direction, LidarObservation, LinkObservation,
    decide, observe_frame,
)
from lidar import Lidar, LidarReport, distance_bucket
from stm32 import STM32, Stm32Report, VALID_DIRECTIONS, VALID_SPEEDS
from telemetry import (
    LOG_CAT_CAMERA, LOG_CAT_LIDAR, LOG_CAT_STM32, LOG_CAT_SYSTEM,
    SensorProbe, TelemetryPublisher, TelemetryState,
)

# ── Logging ─────────────────────────────────────────────────────────────
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
logging.getLogger('werkzeug').setLevel(logging.WARNING)


# ── Helpers ────────────────────────────────────────────────────────────

def probe_to_dict(report) -> dict:
    if isinstance(report, CameraReport):
        return SensorProbe(
            device='camera',
            port='CSI (IMX219 ribbon)' if report.pipeline else 'fake',
            status='OK' if report.frame_ok else ('NO_FRAMES' if report.opened else 'FAILED'),
            error=report.error,
            latency_ms=report.latency_ms,
        ).as_dict()
    if isinstance(report, LidarReport):
        return SensorProbe(
            device='lidar', port=report.port,
            status='OK' if report.opened else 'FAILED',
            error=report.error, latency_ms=report.latency_ms,
            serial=report.serial,
        ).as_dict()
    if isinstance(report, Stm32Report):
        return SensorProbe(
            device='stm32', port=report.port,
            status='OK' if report.opened else 'FAILED',
            error=report.error, latency_ms=report.latency_ms,
        ).as_dict()
    return {}


# ── Orchestrator ───────────────────────────────────────────────────────

class AGV:
    """Thin controller that wires Camera + Lidar + STM32 + Telemetry."""

    def __init__(self) -> None:
        self.camera = Camera()
        self.lidar = Lidar()
        self.stm32 = STM32()
        self.telemetry = TelemetryPublisher()
        self.probes: list[dict] = []
        self.last_error: Optional[str] = None
        self.last_camera_state: Optional[str] = None
        self.last_camera_connection: Optional[bool] = None
        self.last_lidar_state: Optional[tuple[str, int]] = None
        self.last_lidar_connection: Optional[bool] = None
        self.last_stm32_connection: Optional[bool] = None
        self.last_final_cmd: Optional[str] = None
        self.last_speed_sent = None
        self.last_cmd: Optional[str] = None
        self.stop_reason = 'main loop ended'

    # ── startup ──────────────────────────────────────────────────────
    def probe(self) -> None:
        self.probes = [
            probe_to_dict(self.camera.open()),
            probe_to_dict(self.lidar.probe()),
            probe_to_dict(self.stm32.connect()),
        ]

    def start(self) -> None:
        self.telemetry.restore_previous()
        self.telemetry.start()
        log.info('=' * 55)
        log.info('  AGV MAIN BRAIN — Unified Control System')
        log.info('  LiDAR + Camera → Jetson Nano → STM32')
        log.info('=' * 55)
        self.lidar.start()
        self.probe()
        log.info('[Probe] Camera: %s', self.probes[0])
        log.info('[Probe] STM32:   %s', self.probes[2])
        log.info('[Probe] LiDAR:   %s', self.probes[1])
        log.info('Camera running %s — Press Ctrl+C to stop',
                 'headlessly' if not SHOW_WINDOW else 'with local window')

    # ── shutdown ─────────────────────────────────────────────────────
    def stop(self) -> None:
        log.warning('Shutting down AGV...')
        if self.stm32.isOpen():
            self.stm32.send('S')
            log.info('   Motors stopped.')
        self.camera.release()
        self.lidar.close()
        self.stm32.close()
        log.info('AGV shutdown complete.')
        self.telemetry.publish(
            TelemetryState(
                running=False,
                sensor_probes=self.probes,
                last_error=self.last_error,
                final_command='S',
            ),
            log_message=[
                (LOG_CAT_CAMERA, 'Camera disconnected (capture closed)'),
                (LOG_CAT_LIDAR,  'LiDAR disconnected'),
                (LOG_CAT_STM32,  'STM32 serial link disconnected'),
                (LOG_CAT_SYSTEM, f'AGV stopped ({self.stop_reason})'),
            ],
        )
        self.telemetry.stop()

    # ── main loop ────────────────────────────────────────────────────
    def tick(self) -> bool:
        """Process one iteration; return False when the loop should stop."""
        frame_info = self.camera.latest()
        frame = frame_info.image if frame_info else None

        line_obs = observe_frame(frame)
        lidar_state = self.lidar.snapshot()
        lidar_obs = LidarObservation(
            status=lidar_state.status,
            distance_mm=lidar_state.distance,
            connected=lidar_state.connected,
        )
        ack_age = self.stm32.ack_age_ms()
        link_obs = LinkObservation(
            stm32_alive=self.stm32.link_alive(),
            ack_age_ms=ack_age,
        )

        d = decide(line_obs, lidar_obs, link_obs)

        # Draw the HUD on a copy of the frame for the dashboard.
        publish_frame: Optional[np.ndarray] = None
        if frame is not None:
            output = frame.copy()
            self._draw_hud(output, line_obs.direction, d)
            publish_frame = output
        else:
            placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
            self._draw_placeholder(placeholder, d)
            publish_frame = placeholder

        # Send to STM32 (only on change).
        if d.final_cmd != self.last_cmd:
            if d.final_cmd in VALID_DIRECTIONS:
                if d.final_cmd == 'F' and d.speed != self.last_speed_sent:
                    self.stm32.send(d.speed) if d.speed in VALID_SPEEDS else None
                    self.last_speed_sent = d.speed
                self.stm32.send(d.final_cmd)
                self.last_cmd = d.final_cmd

        # Emit transition events.
        events = self._transitions(line_obs, lidar_state, d)
        log_msg = events if events else None

        # Publish telemetry.
        state = TelemetryState(
            running=True,
            camera_connected=self.camera.connected,
            camera_backend=self.camera.backend,
            stm32_connected=self.stm32.connected,
            stm32_ack_age_ms=ack_age,
            stm32_encoder_left=self.stm32.encoder_left,
            stm32_encoder_right=self.stm32.encoder_right,
            stm32_encoder_delta_left=self.stm32.encoder_delta_left,
            stm32_encoder_delta_right=self.stm32.encoder_delta_right,
            stm32_firmware_version=self.stm32.firmware_version,
            camera_direction=line_obs.direction.value,
            camera_command=d.camera_cmd,
            lidar_status=lidar_state.status,
            lidar_connected=lidar_state.connected,
            lidar_error=lidar_state.error,
            lidar_distance_mm=lidar_state.distance,
            final_command=d.final_cmd,
            decision_reason=d.status_text,
            sensor_probes=self.probes,
            last_error=self.last_error,
        )
        self.telemetry.publish(state, frame=publish_frame,
                                log_message=log_msg)
        return True

    # ── HUD rendering ────────────────────────────────────────────────
    def _draw_hud(self, frame: np.ndarray, direction: Direction,
                  decision) -> None:
        h, w = frame.shape[:2]
        cv2 = __import__('cv2', fromlist=['cv2'])
        # Highlight detected line.
        line_obs = observe_frame(frame)
        if line_obs.centroid_xy is not None:
            cx, cy = line_obs.centroid_xy
            cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
        # Status bar.
        cv2.rectangle(frame, (0, 0), (w, 60), (0, 0, 0), -1)
        cv2.putText(frame, f"CMD: {decision.final_cmd} | {decision.status_text}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    decision.status_color, 2)
        lidar_color = {'CLEAR': (0, 255, 0),
                       'SLOW': (0, 255, 255),
                       'STOP': (0, 0, 255)}.get(
            self.lidar.snapshot().status, (128, 128, 128))
        cv2.circle(frame, (w - 40, 30), 20, lidar_color, -1)

    def _draw_placeholder(self, frame: np.ndarray, decision) -> None:
        cv2 = __import__('cv2', fromlist=['cv2'])
        h, w = frame.shape[:2]
        cv2.putText(frame, 'NO CAMERA FRAME',
                    (w // 2 - 280, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.putText(frame, f'CMD: {decision.final_cmd} | {decision.status_text}',
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        if self.last_error:
            err = self.last_error[:120]
            cv2.putText(frame, err, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # ── state-transition detection ───────────────────────────────────
    def _transitions(self, line_obs, lidar_state, decision) -> list[tuple[str, str]]:
        events: list[tuple[str, str]] = []
        camera_state = line_obs.direction.value
        if self.last_camera_state is None:
            events.append((LOG_CAT_CAMERA, f'Camera state: {camera_state}'))
        elif camera_state != self.last_camera_state:
            events.append((LOG_CAT_CAMERA,
                           f'Camera: {self.last_camera_state} → {camera_state}'))

        if self.last_camera_connection is None:
            events.append((LOG_CAT_CAMERA,
                           'Camera connected' if self.camera.connected
                           else 'Camera disconnected'))
        elif self.camera.connected != self.last_camera_connection:
            events.append((LOG_CAT_CAMERA,
                           'Camera connected' if self.camera.connected
                           else 'Camera disconnected'))

        if self.last_stm32_connection is None:
            events.append((LOG_CAT_STM32,
                           'STM32 connected' if self.stm32.connected
                           else 'STM32 disconnected'))
        elif self.stm32.connected != self.last_stm32_connection:
            events.append((LOG_CAT_STM32,
                           'STM32 connected' if self.stm32.connected
                           else 'STM32 disconnected'))

        if self.last_lidar_connection is None:
            events.append((LOG_CAT_LIDAR,
                           f"LiDAR {lidar_state.status}"))
        elif lidar_state.connected != self.last_lidar_connection:
            events.append((LOG_CAT_LIDAR,
                           'LiDAR connected' if lidar_state.connected
                           else f"LiDAR disconnected: {lidar_state.error or ''}"))

        cur_lidar = (lidar_state.status,
                     distance_bucket(lidar_state.distance))
        if self.last_lidar_state is None:
            events.append((LOG_CAT_LIDAR,
                           f"LiDAR state: {cur_lidar[0]}"))
        elif cur_lidar != self.last_lidar_state:
            old_band = (f"{self.last_lidar_state[1] * 250}-"
                        f"{(self.last_lidar_state[1] + 1) * 250} mm"
                        if self.last_lidar_state[1] else 'no obstacle')
            new_dist = (f"{lidar_state.distance:.0f} mm"
                        if lidar_state.distance else 'no obstacle')
            events.append((LOG_CAT_LIDAR,
                           f"LiDAR: {self.last_lidar_state[0]} {old_band} → "
                           f"{cur_lidar[0]} {new_dist}"))

        if self.last_final_cmd is None:
            events.append((LOG_CAT_STM32,
                           f"Final command: {decision.final_cmd}"))
        elif decision.final_cmd != self.last_final_cmd:
            events.append((LOG_CAT_STM32,
                           f"Final command: {self.last_final_cmd} → "
                           f"{decision.final_cmd}"))

        # Update tracking vars.
        self.last_camera_state = camera_state
        self.last_camera_connection = self.camera.connected
        self.last_stm32_connection = self.stm32.connected
        self.last_lidar_connection = lidar_state.connected
        self.last_lidar_state = cur_lidar
        self.last_final_cmd = decision.final_cmd
        return events


# ── Entry point ────────────────────────────────────────────────────────

def main() -> None:
    agv = AGV()
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        agv.start()
        while True:
            try:
                if not agv.tick():
                    break
            except KeyboardInterrupt:
                agv.stop_reason = 'stop requested by user'
                break
            except Exception as exc:
                tb = traceback.format_exc()
                agv.last_error = f'{type(exc).__name__}: {exc}'
                log.error('[MainLoop] Unhandled exception — staying alive\n%s', tb)
                time.sleep(0.5)
    except KeyboardInterrupt:
        agv.stop_reason = 'stop requested by user'
    finally:
        agv.stop()


if __name__ == '__main__':
    main()