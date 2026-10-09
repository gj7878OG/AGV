# AGV Web Dashboard Plan

## Objective
Create a real-time web-based interface (Dashboard) hosted on the Jetson Nano to monitor the AGV's state, view the live camera feed, and read decision logs from any laptop/device on the same WiFi network.

## Proposed Architecture
The Dashboard will be integrated directly into our `agv_main.py` script using **Flask** (a lightweight Python web framework). 

1. **Backend (Flask Server):**
   - Runs in a separate background thread inside `agv_main.py`.
   - Serves the HTML/CSS/JS frontend.
   - Provides a continuous MJPEG video stream from the OpenCV frames.
   - Provides an API endpoint (e.g., `/status`) to fetch the latest decision logs, LiDAR state, and current command.

2. **Frontend (Web Browser):**
   - **Video Player:** An `<img>` tag that displays the MJPEG stream.
   - **Status Panel:** Displays current Camera decision (L/R/F/S), LiDAR status (CLEAR/SLOW/STOP), and the final STM32 command.
   - **Log Window:** A scrolling text box that fetches new log entries via AJAX and appends them to the view.

## Implementation Steps

### Step 1: Install Dependencies
We will need to install Flask on the Jetson Nano:
```bash
pip3 install Flask
```

### Step 2: Modify `agv_main.py`
- Add Flask app initialization.
- Create a shared thread-safe queue or list to hold the last ~50 log messages.
- Create a route `/video_feed` that uses a generator to yield OpenCV frames encoded as JPEG.
- Create a route `/status` that returns the current state and new logs as JSON.
- Start the Flask app using `app.run(host='0.0.0.0', port=5000)` in a new `daemon=True` thread so it doesn't block the OpenCV main loop.

### Step 3: Create the HTML Template
Create a `templates/index.html` file that includes:
- A clean, dark-mode CSS layout.
- The video feed container.
- JavaScript `setInterval` function to fetch `/status` every 500ms and update the UI/logs.

## Future Enhancements
- **Manual Override:** Add buttons on the dashboard to pause autonomy and manually drive the AGV (F/B/L/R/S).
- **Sensor Graphs:** Add Chart.js to plot LiDAR distances over time.
