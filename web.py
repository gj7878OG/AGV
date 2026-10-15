"""Web dashboard for AGV telemetry.

Two surfaces, both reading the same telemetry directory:

  - HTTP / Flask (default port 5000):
      /            →  static dashboard page
      /status      →  JSON snapshot (used for first paint)
      /video_feed  →  MJPEG stream of the latest annotated frame
  - WebSocket (default port 5001):
      /            →  push the live status every ~500 ms
        Clients (a single browser tab) connect once and receive updates
        instead of polling. Lower CPU on the Jetson and smoother UI.

The HTTP layer is also a tiny server for the WebSocket — `flask run`
is not involved.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, render_template, request

from config import (
    DASHBOARD_HOST, DASHBOARD_HTTP_PORT, DASHBOARD_WS_PORT, TELEMETRY_DIR,
)

log = logging.getLogger('agv.web')

STATUS_PATH = Path(TELEMETRY_DIR) / 'status.json'
FRAME_PATH = Path(TELEMETRY_DIR) / 'frame.jpg'

# ─────────────────────────────────────────────────────────────────
# HTTP layer (Flask)
# ─────────────────────────────────────────────────────────────────

flask_app = Flask('agv-dashboard')


@flask_app.context_processor
def inject_project_config():
    """Make the project's `config.py` available to every template as
    `project_config`. Without this, `{{ config.X }}` in the template would
    resolve to Flask's own config object, not the AGV config module.
    """
    import config as _project_config
    return {'project_config': _project_config}


def _read_status() -> dict:
    fallback = {
        'running': False,
        'camera_connected': None,
        'camera_backend': 'argus',
        'stm32_connected': None,
        'stm32_ack_age_ms': 0.0,
        'stm32_encoder_left': 0,
        'stm32_encoder_right': 0,
        'stm32_encoder_delta_left': 0,
        'stm32_encoder_delta_right': 0,
        'stm32_firmware_version': '',
        'camera_direction': 'UNAVAILABLE',
        'camera_command': 'S',
        'lidar_status': 'UNKNOWN',
        'lidar_connected': None,
        'lidar_error': None,
        'lidar_distance_mm': 0,
        'final_command': 'S',
        'decision_reason': '',
        'updated_at': None,
        'logs': [],
        'sensor_probes': [],
        'last_error': None,
    }
    try:
        with STATUS_PATH.open('r', encoding='utf-8') as fh:
            payload = json.load(fh)
        for key, default in fallback.items():
            payload.setdefault(key, default)
        return payload
    except (OSError, ValueError):
        return fallback


@flask_app.route('/', methods=['GET'])
def index():
    return render_template('dashboard.html')


@flask_app.route('/status', methods=['GET'])
def status():
    payload = _read_status()
    try:
        after_id = int(request.args.get('after', '0'))
    except ValueError:
        after_id = 0
    filtered = []
    for entry in payload.get('logs', []):
        if entry.get('id', 0) <= after_id:
            continue
        entry.setdefault('category', 'SYSTEM')
        filtered.append(entry)
    payload['logs'] = filtered
    return jsonify(payload)


def _generate_frames():
    while True:
        try:
            with FRAME_PATH.open('rb') as fh:
                frame = fh.read()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Cache-Control: no-cache\r\n\r\n' + frame + b'\r\n')
        except OSError:
            pass
        time.sleep(0.1)


@flask_app.route('/video_feed', methods=['GET'])
def video_feed():
    return Response(
        _generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-cache, no-store, must-revalidate'},
    )


# ─────────────────────────────────────────────────────────────────
# WebSocket layer (push instead of polling)
# ─────────────────────────────────────────────────────────────────

class WSHub:
    """Tracks active WebSocket clients and broadcasts status snapshots."""

    def __init__(self) -> None:
        self._clients: set = set()
        self._lock = asyncio.Lock()

    async def register(self, ws) -> None:
        async with self._lock:
            self._clients.add(ws)

    async def unregister(self, ws) -> None:
        async with self._lock:
            self._clients.discard(ws)

    async def broadcast(self, message: dict) -> None:
        async with self._lock:
            clients = list(self._clients)
        if not clients:
            return
        encoded = json.dumps(message)
        dead = []
        for ws in clients:
            try:
                await ws.send(encoded)
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.discard(ws)


HUB = WSHub()


async def _ws_handler(ws) -> None:
    await HUB.register(ws)
    try:
        # Send first snapshot immediately, then await incoming pings.
        snapshot = _read_status()
        await ws.send(json.dumps(snapshot))
        async for _ in ws:
            pass  # client messages are unused; the server pushes only
    finally:
        await HUB.unregister(ws)


async def _publisher_loop() -> None:
    """Push status.json snapshots to every connected WS client at ~2 Hz."""
    last_id = 0
    while True:
        try:
            payload = _read_status()
            for entry in payload.get('logs', []):
                if entry.get('id', 0) > last_id:
                    last_id = entry['id']
            await HUB.broadcast(payload)
        except Exception as exc:
            log.warning('[WS] publish error: %s', exc)
        await asyncio.sleep(0.5)


async def _ws_main() -> None:
    import websockets
    async with websockets.serve(_ws_handler, DASHBOARD_HOST,
                                 DASHBOARD_WS_PORT,
                                 ping_interval=20, ping_timeout=20):
        await _publisher_loop()


def _start_ws() -> None:
    """Run the asyncio WebSocket loop on a dedicated daemon thread."""
    def _run():
        try:
            asyncio.run(_ws_main())
        except Exception as exc:
            log.warning('[WS] server stopped: %s', exc)
    threading.Thread(target=_run, name='ws-hub', daemon=True).start()


# ─────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────

def main() -> None:
    _start_ws()
    flask_app.run(
        host=DASHBOARD_HOST,
        port=DASHBOARD_HTTP_PORT,
        threaded=True,
        debug=False,
        use_reloader=False,
    )


if __name__ == '__main__':
    main()