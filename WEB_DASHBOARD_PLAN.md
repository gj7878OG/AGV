# AGV Web Dashboard Plan

**Implementation status:** The read-only dashboard and telemetry publisher are
implemented in this branch. Jetson hardware and second-device network
validation are still required.

## Objective

Provide a browser-based, read-only view of AGV operation from a laptop or
phone on the same Wi-Fi network. The dashboard will show the annotated camera
feed, camera steering decision, LiDAR status and distance, final STM32 command,
and recent decision logs.

## Current project basis

`agv_main.py` currently runs the camera line detector, LiDAR safety logic, and
STM32 command output in one control process. It already has the data needed by
the dashboard. The camera loop calls `cv2.imshow()` and `cv2.waitKey()`;
these are optional so the AGV can run without a local desktop display.

The software paths are present, but hardware operation must be verified on the
Jetson before describing the integrated camera, LiDAR, and motor-control system
as field tested.

## Proposed architecture

Keep the dashboard in a separate process from `agv_main.py` so a web-server
failure or restart does not stop the control loop. The AGV process publishes
read-only telemetry and the latest annotated JPEG to a private local runtime
directory. The dashboard process reads those files and serves the page and API
with Flask.

```text
Camera + LiDAR
      ↓
agv_main.py ── atomically updated status/frame files ──> Flask dashboard
      ↓                                                  ↓
STM32 commands                                      Browser on Wi-Fi
```

Use a configurable runtime directory, `/tmp/agv-dashboard/` by default. Write
temporary files and atomically replace published files so the dashboard never
reads a partially written JSON document or JPEG. Publish an update timestamp
so the page can indicate stale data if the control process stops updating.

The dashboard must not send commands to the STM32. Bind its web server to the
Jetson network interface so other devices on the local network can connect;
do not expose the development server directly to the public internet.

## Dashboard behavior

- `/` serves the dashboard page.
- `/video_feed` serves the latest annotated frame as an MJPEG stream.
- `/status` returns JSON containing camera direction and command, LiDAR link,
  state and distance, final STM32 command, timestamp, and recent log entries.
- Logs have monotonically increasing IDs and retain the latest 500 events in
  the shared status file across control-process restarts. Clients pass the last
  seen ID when polling so multiple browsers do not consume each other's entries.
- Log startup/shutdown, STM32 and LiDAR connection problems, camera state,
  camera direction changes, LiDAR state changes, and final command changes.
  LiDAR distance transition entries are grouped in 250 mm bands to avoid
  logging sensor noise on every frame.
- The browser polls status about twice per second and shows whether data is
  stale. Video and status remain display-only.
- The state panel shows camera capture, LiDAR scanner, and STM32 serial-link
  status explicitly as connected or disconnected.
- STM32 status reflects serial open/write success; the current firmware does
  not acknowledge commands, so it cannot confirm the MCU is executing them.

The AGV loop publishes one latest frame at up to 5 fps using a background
writer. The dashboard process streams that JPEG without opening a second camera
capture. Keep the published event history bounded (500 entries); the page shows
these in a scrollable, newest-first log.

## Implementation and validation status

1. **Implemented:** `agv_main.py` publishes the current
   annotated frame, camera decision, LiDAR readings, final command, timestamp,
   and bounded event history using atomic file replacement on a background
   telemetry writer.
2. **Implemented:** `cv2.imshow()` and `cv2.waitKey()` are conditional on
   `AGV_SHOW_WINDOW`; headless operation is the default without `DISPLAY`.
3. **Implemented:** `dashboard.py` and `templates/dashboard.html` read
   telemetry files and do not open serial ports or control the vehicle.
4. **Implemented:** Dependencies and startup instructions are documented in
   this file and `AGV_README.md`.
5. **Pending hardware validation:** Confirm page, status freshness, and video
   from the Jetson and a second device on the same Wi-Fi. Verify that stopping
   or restarting the dashboard does not stop `agv_main.py`.

## Dependencies and operation

The existing control program lists `pyserial` and `rplidar-roboticia`; it also
uses OpenCV and NumPy. The dashboard adds Flask. Keep dashboard dependencies
separate from the control process where practical so the web UI can be
installed or restarted independently.

Create an isolated dashboard environment and install Flask. On the Jetson's
older Python releases, upgrade the packaging tools first so `pip` can select
the available ARM wheels:

```bash
python3 -m venv .venv-dashboard
. .venv-dashboard/bin/activate
python -m pip install --upgrade 'pip<22' 'setuptools<60' wheel
python -m pip install -r requirements-dashboard.txt
```

Run the AGV control loop and dashboard in separate terminals under the same
user so both can access the telemetry directory:

```bash
python3 agv_main.py
```

```bash
. .venv-dashboard/bin/activate
python dashboard.py
```

Open `http://<jetson-lan-ip>:5000` from another device on the same network.
The default shared directory is `/tmp/agv-dashboard`; set
`AGV_DASHBOARD_DIR` in both processes to use another location. Set
`AGV_SHOW_WINDOW=1` to enable the local OpenCV window; by default it is
headless when `DISPLAY` is not set.

The dashboard records state transitions rather than every video frame. It
keeps the newest 500 events in `status.json` so the page remains useful without
creating an indefinitely growing log file.

## Out of scope for the first version

- Manual drive or pause controls. These can issue motor commands and require a
  separate safety and access-control design.
- Sensor graphs and historical storage. Consider these after the live status
  view is reliable.
