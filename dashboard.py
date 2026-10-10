#!/usr/bin/env python3
"""Read-only web dashboard for telemetry published by agv_main.py."""

import json
import os
import time

from flask import Flask, Response, jsonify, render_template, request


TELEMETRY_DIR = os.environ.get('AGV_DASHBOARD_DIR', '/tmp/agv-dashboard')
STATUS_PATH = os.path.join(TELEMETRY_DIR, 'status.json')
FRAME_PATH = os.path.join(TELEMETRY_DIR, 'frame.jpg')
HOST = os.environ.get('AGV_DASHBOARD_HOST', '0.0.0.0')
PORT = int(os.environ.get('AGV_DASHBOARD_PORT', '5000'))

app = Flask(__name__)


def read_status():
    try:
        with open(STATUS_PATH, 'r', encoding='utf-8') as status_file:
            payload = json.load(status_file)
        payload.setdefault('camera_connected', None)
        payload.setdefault('lidar_connected', None)
        payload.setdefault('stm32_connected', None)
        return payload
    except (OSError, ValueError):
        return {
            'running': False,
            'camera_connected': None,
            'stm32_connected': None,
            'camera_direction': 'UNAVAILABLE',
            'camera_command': 'S',
            'lidar_status': 'UNKNOWN',
            'lidar_connected': None,
            'lidar_error': None,
            'lidar_distance_mm': 0,
            'final_command': 'S',
            'updated_at': None,
            'logs': [],
            'sensor_probes': [],
            'last_error': None,
        }


@app.route('/', methods=['GET'])
def index():
    return render_template('dashboard.html')


@app.route('/status', methods=['GET'])
def status():
    payload = read_status()
    try:
        after_id = int(request.args.get('after', '0'))
    except ValueError:
        after_id = 0
    payload['logs'] = [entry for entry in payload.get('logs', [])
                       if entry.get('id', 0) > after_id]
    return jsonify(payload)


def generate_frames():
    """Stream the newest complete JPEG file; never access the camera device."""
    while True:
        try:
            with open(FRAME_PATH, 'rb') as frame_file:
                frame = frame_file.read()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Cache-Control: no-cache\r\n\r\n' + frame + b'\r\n')
        except OSError:
            pass
        time.sleep(0.1)


@app.route('/video_feed', methods=['GET'])
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-cache, no-store, must-revalidate'},
    )


if __name__ == '__main__':
    # The dashboard is read-only and separate from motor/camera control.
    app.run(host=HOST, port=PORT, threaded=True, debug=False, use_reloader=False)
