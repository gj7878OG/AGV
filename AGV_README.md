# Autonomous Guided Vehicle (AGV) --- Jetson Nano Computer Vision Platform

> **Implementation status:** `agv_main.py` is the current unified prototype.
> It contains camera line detection, LiDAR safety logic, and STM32 serial
> commands. Hardware validation is a separate step; the presence of code does
> not mean the integrated system has been tested on the vehicle. The browser
> dashboard is planned in [WEB_DASHBOARD_PLAN.md](WEB_DASHBOARD_PLAN.md) and is
> not implemented yet.

## 1. Project Overview

This project is the development of a **camera-guided Autonomous Guided
Vehicle (AGV)** intended to navigate along a marked path while detecting
people/obstacles and eventually controlling its drive system
autonomously.

The current development strategy is deliberately incremental:

1.  Bring up the Jetson Nano platform.
2.  Establish reliable camera capture.
3.  Detect the navigation line using computer vision.
4.  Determine the line position and steering direction.
5.  Add human/obstacle detection.
6.  Integrate LiDAR for range/obstacle information.
7.  Connect perception decisions to the motor controller.
8.  Validate the complete AGV on the physical chassis.

The present software prototype is being developed directly on the
**Jetson Nano**, without the previously considered ESP32-CAM dependency.

------------------------------------------------------------------------

## 2. Project Goals

### Primary goals

-   Build a functional autonomous guided vehicle.
-   Use a camera to detect a colored navigation line.
-   Follow the line using real-time computer vision.
-   Detect people/obstacles in the vehicle's path.
-   Slow down or stop when an obstacle is detected.
-   Use LiDAR as an additional sensing mechanism.
-   Eventually convert perception output into motor-control commands.
-   Demonstrate the system in a controlled indoor environment.

### Demonstration goal

The initial demonstration does not require a fully autonomous physical
vehicle.

The first CV demonstration is intended to show:

``` text
Camera
   ↓
Image Processing
   ↓
Yellow Line Detection
   ↓
Line Detected
   ↓
Direction / Steering Decision
```

and later:

``` text
Camera / LiDAR
      ↓
Human / Obstacle Detection
      ↓
Slow Down / Stop
```

This staged approach allows the computer-vision subsystem to be
validated before connecting it to motors.

------------------------------------------------------------------------

# 3. Current System Architecture

The current architecture is centered around the Jetson Nano.

``` text
                    ┌──────────────────────┐
                    │      Camera          │
                    │     IMX219 CSI       │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │      Jetson Nano     │
                    │                      │
                    │  Camera Capture      │
                    │        ↓             │
                    │  OpenCV Processing   │
                    │        ↓             │
                    │  Line Detection      │
                    │        ↓             │
                    │ Direction Decision   │
                    └──────────┬───────────┘
                               │
                 ┌─────────────┴─────────────┐
                 │                           │
                 ▼                           ▼
        ┌────────────────┐          ┌────────────────┐
        │ Motor Control  │          │ Obstacle /     │
        │     Future     │          │ Human Detection│
        └────────────────┘          └───────┬────────┘
                                            │
                                            ▼
                                      ┌────────────┐
                                      │   LiDAR    │
                                      │   Future   │
                                      └────────────┘
```

------------------------------------------------------------------------

# 4. Hardware Architecture

## 4.1 Main Controller --- Jetson Nano

The Jetson Nano is the primary computing platform.

Responsibilities:

-   Camera interfacing
-   Image acquisition
-   OpenCV processing
-   Yellow-line detection
-   Future human/object detection
-   Future sensor fusion
-   High-level navigation decisions
-   Future communication with motor-control hardware

The Jetson was selected because the project requires substantially more
processing than a conventional microcontroller-based line follower once
computer vision and object detection are introduced.

------------------------------------------------------------------------

## 4.2 Camera

The currently identified camera is:

**Sony IMX219 CSI camera**

The Jetson detects it as:

``` text
vi-output, imx219 7-0010
    /dev/video0
```

This confirms that the camera driver/device is being detected by Linux.

The camera is intended to provide the primary visual input for:

-   Navigation-line detection
-   Future human detection
-   Future scene understanding

------------------------------------------------------------------------

## 4.3 LiDAR

LiDAR is part of the planned final AGV sensing architecture.

Expected responsibilities:

-   Measure distance to objects
-   Detect obstacles outside the camera's reliable visual range
-   Provide additional safety information
-   Support obstacle avoidance / stopping logic

LiDAR integration has not yet been completed in the current software
stage.

------------------------------------------------------------------------

## 4.4 Chassis

The planned platform is a:

-   Four-wheel AGV chassis
-   Camera mounted above/front of the vehicle
-   LiDAR mounted at an elevated/rotatable position
-   Motor-driven wheels

The exact motor driver and final electrical wiring are still part of the
hardware-integration phase.

------------------------------------------------------------------------

## 4.5 Wheel Encoders

Wheel encoders were considered because they would provide:

-   Wheel-speed feedback
-   Distance estimation
-   Odometry
-   More accurate closed-loop motor control

However, encoders are not currently available on the chassis.

The first implementation can therefore operate without wheel encoders,
while keeping encoder integration as a possible future improvement.

------------------------------------------------------------------------

# 5. Controller Architecture Decision

An important architectural decision was made during development:

## ESP32-CAM removed from the current CV architecture

The initial project concept considered using multiple processing
devices, including an ESP32-CAM.

For the current computer-vision phase, this was simplified to:

``` text
Camera → Jetson Nano → OpenCV
```

instead of:

``` text
Camera → ESP32-CAM → Communication → Main Computer
```

### Why?

The Jetson Nano can directly perform:

-   Camera capture
-   OpenCV processing
-   Color segmentation
-   Contour detection
-   Line tracking
-   Future neural-network inference

Removing an intermediate camera processor reduces:

-   Communication complexity
-   Software duplication
-   Synchronization problems
-   Debugging overhead

The ESP32 is therefore not required for the current CV demonstration.

A separate microcontroller can still be considered later if the final
motor-control design benefits from a dedicated real-time controller.

------------------------------------------------------------------------

# 6. Navigation Environment

The original navigation concept considered colored tape on a gray
concrete floor.

The project has now moved toward using:

> **Yellow tape/line on a gray floor**

The yellow line is intended to provide strong color contrast for
HSV-based segmentation.

The expected environment is controlled indoor navigation.

------------------------------------------------------------------------

# 7. Computer Vision Pipeline

The planned line-following pipeline is:

``` text
Camera Frame
     ↓
Resize / Preprocessing
     ↓
Convert BGR → HSV
     ↓
Yellow Color Threshold
     ↓
Binary Mask
     ↓
Noise Removal
     ↓
Contour Detection
     ↓
Largest / Relevant Line Region
     ↓
Centroid Calculation
     ↓
Compare With Image Center
     ↓
Steering Decision
```

For example:

``` text
             Camera Image
       ┌─────────────────────┐
       │                     │
       │        YELLOW       │
       │          │          │
       │          │          │
       │          │          │
       │          ▼          │
       │        LINE         │
       │                     │
       └─────────────────────┘
                 │
                 ▼
        Calculate line center
                 │
        ┌────────┼────────┐
        ▼        ▼        ▼
      LEFT    CENTER    RIGHT
        │        │        │
        ▼        ▼        ▼
    Turn Left  Forward  Turn Right
```

------------------------------------------------------------------------

# 8. Yellow-Line Detection

The selected color-processing method is based on the HSV color space.

A typical implementation will:

1.  Capture a frame.
2.  Convert the frame from BGR to HSV.
3.  Apply a lower and upper HSV threshold for yellow.
4.  Produce a binary mask.
5.  Apply morphological operations if necessary.
6.  Find contours.
7.  Select the relevant line contour.
8.  Calculate its center.
9.  Compare the center against the camera frame center.

Conceptually:

``` python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(
    hsv,
    yellow_lower,
    yellow_upper
)

contours, _ = cv2.findContours(
    mask,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)
```

The exact HSV thresholds should be calibrated using the actual yellow
tape, camera exposure, floor material, and lighting conditions.

Hard-coded values should therefore be treated as starting points rather
than final production values.

------------------------------------------------------------------------

# 9. Steering Decision

Once the line centroid is known:

``` text
frame_center = image_width / 2
line_center  = detected_line_centroid_x
error        = line_center - frame_center
```

The error can be converted into a basic steering command.

Example:

``` text
error < negative threshold
        ↓
     LEFT

error within threshold
        ↓
    STRAIGHT

error > positive threshold
        ↓
     RIGHT
```

A more advanced controller can later use proportional control:

``` text
steering = Kp × error
```

and eventually:

``` text
steering = Kp × error
         + Ki × accumulated_error
         + Kd × rate_of_error_change
```

A PID controller should only be introduced after reliable line detection
and motor response have been established.

------------------------------------------------------------------------

# 10. Planned Human / Obstacle Detection

Human/obstacle detection is a separate stage from basic line detection.

The intended behavior is:

``` text
Person / obstacle detected
          ↓
Estimate distance / position
          ↓
       Slow down
          ↓
If obstacle too close
          ↓
          STOP
```

Possible perception sources:

-   Camera-based object detection
-   LiDAR distance measurement
-   Combined camera + LiDAR logic

The first project demonstration can use a simple conceptual output:

``` text
LINE DETECTED
HUMAN DETECTED
SLOWING DOWN
```

The final vehicle should use measured distance and safety thresholds
rather than only a visual label.

------------------------------------------------------------------------

# 11. Planned Sensor Fusion

The final architecture can combine camera and LiDAR:

``` text
             ┌─────────────┐
             │   Camera    │
             └──────┬──────┘
                    │
                    ▼
             Visual Detection
                    │
                    │
             ┌──────┴──────┐
             │             │
             ▼             ▼
       Line Position   Object Detection
             │             │
             └──────┬──────┘
                    │
             ┌──────▼──────┐
             │ Sensor Fusion│
             └──────┬──────┘
                    ▲
                    │
             ┌──────┴──────┐
             │    LiDAR    │
             └─────────────┘
                    │
                    ▼
             Safety Decision
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
       Navigate             Stop
```

The camera is useful for understanding the path and visual objects.

LiDAR is useful for direct range measurements.

Using both provides a more robust perception system than relying on a
single sensor.

------------------------------------------------------------------------

# 12. Current Project Files

The current project directory includes:

``` text
agv/
├── agv_main.py
├── main.c
├── main.h
├── camera_test.py
├── manual_motor_test.py
├── dashboard.py
├── requirements-dashboard.txt
├── templates/dashboard.html
├── yellow_line.py
└── WEB_DASHBOARD_PLAN.md
```

`agv_main.py` is the unified camera + LiDAR + STM32 prototype. The other
scripts are standalone camera, line-following, and LiDAR experiments. The
read-only dashboard is implemented in `dashboard.py` and `templates/`.

## `agv_main.py`

Purpose:

-   Detect and follow the yellow line using the IMX219 camera and OpenCV.
-   Use LiDAR readings for stop and slow safety behavior.
-   Send the final command to the STM32 over serial.

Current status:

**Implemented in source; end-to-end Jetson and vehicle validation must be
confirmed separately.**

### Optional read-only web dashboard

The dashboard runs as a separate process and reads status and annotated frames
published by `agv_main.py`. It does not access the camera or send motor
commands. The activity panel records startup/shutdown, sensor connection
problems, camera direction changes, LiDAR state/distance-band changes, and
final command changes. The state panel shows whether camera capture, LiDAR, and
the STM32 serial link are connected. It keeps the latest 500 events, newest
first.

Create an isolated dashboard environment and install Flask. On older Jetson
Python releases, upgrade the packaging tools first so pip can select the ARM
wheels:

```bash
python3 -m venv .venv-dashboard
. .venv-dashboard/bin/activate
python -m pip install --upgrade 'pip<22' 'setuptools<60' wheel
python -m pip install -r requirements-dashboard.txt
```

Start the control program in one terminal:

```bash
python3 agv_main.py
```

Start the dashboard in another terminal as the same user:

```bash
. .venv-dashboard/bin/activate
python dashboard.py
```

Open `http://<jetson-lan-ip>:5000` from a device on the same Wi-Fi network. Both
processes use `/tmp/agv-dashboard` by default; set `AGV_DASHBOARD_DIR` in both
if using another directory. The dashboard is read-only and intended for a
trusted local network.

## `camera_test.py`

Purpose:

-   Verify camera capture.
-   Display camera frames.
-   Establish the basic camera pipeline before computer vision is added.

Current status:

**Created. An earlier bring-up attempt recorded an Argus/IMX219 capture
failure; recheck the camera on the current Jetson before relying on that
diagnosis.**

------------------------------------------------------------------------

## `yellow_line.py`

Purpose:

-   Prototype yellow-line detection.
-   Process camera frames.
-   Detect the yellow navigation line.
-   Eventually calculate line position and steering direction.

Current status:

**Created as the line-detection stage, but should be tested after
reliable camera capture is restored.**

------------------------------------------------------------------------

# 13. Camera Bring-Up Notes (Historical; Recheck)

The Linux device layer detects the camera:

``` text
/dev/video0
```

`v4l2-ctl` reports:

``` text
vi-output, imx219 7-0010 (platform:54080000.vi:0):
    /dev/video0
```

Therefore:

``` text
Camera device detected          YES
IMX219 driver/device visible    YES
/dev/video0                     YES
Actual frame capture            NO
Argus CaptureSession            FAILING
```

------------------------------------------------------------------------

# 14. Camera Error Recorded During Earlier Bring-Up

The native Jetson GStreamer pipeline was tested:

``` bash
gst-launch-1.0 nvarguscamerasrc ! \
nvvidconv ! \
nvegltransform ! \
nveglglessink
```

It fails with:

``` text
Failed to create CaptureSession
```

The same issue occurs when explicitly specifying:

``` text
sensor-id=0
```

This establishes that the problem is below the Python line-detection
layer.

------------------------------------------------------------------------

# 15. Root Cause Investigation

Kernel logs revealed the most important errors:

``` text
tegra-vii2c 546c0000.i2c: no acknowledge from address 0x10
imx219 7-0010: Error writing mode
vi 54080000.vi: calibration failed with -121 error
```

The sequence indicates:

``` text
Jetson detects IMX219
        ↓
Driver attempts to configure sensor
        ↓
I²C communication with sensor fails
        ↓
Sensor mode cannot be written
        ↓
VI calibration fails
        ↓
Argus cannot create CaptureSession
        ↓
OpenCV cannot receive frames
```

The I²C error is particularly important because the IMX219 uses address
`0x10`.

This means the current issue is most likely related to the **camera
hardware connection, CSI ribbon/camera module communication, or
camera/driver initialization**, rather than the yellow-line algorithm.

------------------------------------------------------------------------

# 16. Immediate Camera Recovery Procedure

Before changing the CV software, the camera connection should be
checked.

### Step 1 --- Power off

``` bash
sudo shutdown -h now
```

Do not disconnect/reconnect the CSI ribbon while the Jetson is powered.

### Step 2 --- Reseat the CSI cable

With the Jetson completely powered off:

1.  Disconnect the CSI ribbon cable.
2.  Check the cable for damage.
3.  Reseat the cable at the Jetson connector.
4.  Reseat the cable at the camera connector.
5.  Ensure it is inserted straight and fully.
6.  Lock both connectors.
7.  Verify the cable orientation for the specific Jetson and camera
    module.

### Step 3 --- Boot again

After powering the Jetson:

``` bash
v4l2-ctl --list-devices
```

The IMX219 should still appear.

### Step 4 --- Restart Argus

``` bash
sudo systemctl restart nvargus-daemon
```

### Step 5 --- Native camera test

``` bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
nvvidconv ! \
nvegltransform ! \
nveglglessink
```

The objective is to obtain a live camera preview.

Only after this succeeds should the Python/OpenCV layer be tested again.

------------------------------------------------------------------------

# 17. Development Milestones

## Phase 1 --- Platform Setup

-   [x] Jetson Nano booted
-   [x] Terminal access established
-   [x] Project directory created
-   [x] Camera device identified
-   [x] IMX219 identified

## Phase 2 --- Camera Bring-Up

-   [x] `/dev/video0` detected
-   [x] IMX219 driver visible
-   [x] GStreamer/Argus pipeline tested
-   [ ] Stable camera frame capture
-   [ ] OpenCV camera preview

## Phase 3 --- Yellow Line Detection

-   [x] Yellow line selected as navigation marker
-   [x] Line-detection script created
-   [ ] Camera frames successfully supplied to OpenCV
-   [ ] HSV threshold calibration
-   [ ] Yellow mask validation
-   [ ] Contour detection
-   [ ] Line centroid calculation
-   [ ] Line direction calculation
-   [ ] Real-time "LINE DETECTED" demonstration

## Phase 4 --- Navigation Logic

-   [ ] Straight detection
-   [ ] Left detection
-   [ ] Right detection
-   [ ] Lost-line detection
-   [ ] Steering error calculation
-   [ ] Smoothing/filtering
-   [ ] Controller tuning

## Phase 5 --- Human / Obstacle Detection

-   [ ] Camera object detection
-   [ ] Human detection
-   [ ] Obstacle classification
-   [ ] Slow-down logic
-   [ ] Stop logic

## Phase 6 --- LiDAR

-   [ ] Identify LiDAR interface
-   [ ] Read distance data
-   [ ] Validate measurements
-   [ ] Define obstacle distance thresholds
-   [ ] Integrate with perception pipeline

## Phase 7 --- Motor Integration

-   [ ] Select motor driver/control interface
-   [ ] Connect motor controller
-   [ ] Test individual motors
-   [ ] Implement forward/reverse
-   [ ] Implement left/right steering
-   [ ] Implement stop
-   [ ] Connect CV steering output to motor commands

## Phase 8 --- Full AGV

-   [ ] Camera mounted and calibrated
-   [ ] LiDAR mounted and calibrated
-   [ ] Line following on physical floor
-   [ ] Obstacle response
-   [ ] Human detection response
-   [ ] Combined sensor operation
-   [ ] Safety testing
-   [ ] Final demonstration

------------------------------------------------------------------------

# 18. Proposed Software Architecture

A clean final software structure can be:

``` text
agv/
│
├── camera/
│   ├── camera_test.py
│   └── camera_pipeline.py
│
├── vision/
│   ├── yellow_line.py
│   ├── line_detector.py
│   └── human_detector.py
│
├── sensors/
│   └── lidar.py
│
├── navigation/
│   ├── steering.py
│   └── obstacle_manager.py
│
├── control/
│   └── motor_controller.py
│
├── config/
│   └── parameters.py
│
├── tests/
│
└── README.md
```

This structure separates:

-   Camera acquisition
-   Computer vision
-   Sensors
-   Navigation
-   Motor control
-   Configuration
-   Testing

This will make the project easier to debug and extend.

------------------------------------------------------------------------

# 19. Navigation State Machine

The final AGV can be represented as a simple state machine:

``` text
                  ┌──────────────┐
                  │    START     │
                  └──────┬───────┘
                         │
                         ▼
                  ┌──────────────┐
                  │ Search Line  │
                  └──────┬───────┘
                         │
                    Line found
                         │
                         ▼
                  ┌──────────────┐
                  │ FOLLOW LINE  │
                  └──────┬───────┘
                         │
             ┌───────────┼───────────┐
             │           │           │
             ▼           ▼           ▼
          LEFT        CENTER       RIGHT
             │           │           │
             └───────────┼───────────┘
                         │
                         ▼
                  ┌──────────────┐
                  │ Check Object │
                  └──────┬───────┘
                         │
                    Obstacle?
                    /       \
                  NO         YES
                  │           │
                  │           ▼
                  │     ┌───────────┐
                  │     │ SLOW/STOP │
                  │     └───────────┘
                  │
                  └──────► FOLLOW
```

------------------------------------------------------------------------

# 20. Safety Logic

The vehicle should not immediately command motors based solely on a
single noisy camera frame.

A safer sequence is:

``` text
Camera frame
    ↓
Detection
    ↓
Validation / filtering
    ↓
Navigation decision
    ↓
Obstacle check
    ↓
Motor command
```

For example:

``` text
Obstacle distance > safe threshold
    → normal line following

Obstacle distance approaching threshold
    → reduce speed

Obstacle distance below stop threshold
    → stop motors
```

The final thresholds should be determined experimentally based on:

-   Vehicle speed
-   Braking distance
-   Sensor latency
-   Camera/LiDAR update rate
-   Floor conditions
-   Vehicle mass

------------------------------------------------------------------------

# 21. Testing Strategy

Testing should proceed from individual components to the complete
system.

## Test 1 --- Camera

Expected result:

``` text
Live camera preview
```

## Test 2 --- Yellow segmentation

Expected result:

``` text
Yellow line → white
Other regions → black
```

## Test 3 --- Line detection

Expected output:

``` text
LINE DETECTED
```

with a visual marker over the detected line.

## Test 4 --- Direction

Move the line relative to the camera:

``` text
Line left of center  → LEFT
Line near center     → STRAIGHT
Line right of center → RIGHT
```

## Test 5 --- Human detection

Place a person in front of the system.

Expected:

``` text
HUMAN DETECTED
SLOWING DOWN
```

## Test 6 --- Obstacle stop

Place an obstacle within the defined stopping distance.

Expected:

``` text
OBSTACLE DETECTED
STOP
```

## Test 7 --- Physical line following

Run the AGV at low speed on the controlled test track.

## Test 8 --- Integrated operation

Test:

``` text
Line following
      +
Human detection
      +
LiDAR obstacle detection
      +
Motor control
```

------------------------------------------------------------------------

# 22. Initial Demo Flow

For a project presentation, the demonstration can be divided into three
levels.

### Demo A --- Computer Vision

Show:

``` text
Camera
  ↓
Yellow line
  ↓
Line detection
  ↓
Direction
```

On-screen labels:

``` text
LINE DETECTED
DIRECTION: LEFT
```

or:

``` text
LINE DETECTED
DIRECTION: STRAIGHT
```

### Demo B --- Human Detection

Show:

``` text
HUMAN DETECTED
SLOWING DOWN
```

### Demo C --- Integrated AGV

Finally demonstrate:

``` text
Camera + LiDAR
      ↓
Perception
      ↓
Navigation
      ↓
Motor Control
      ↓
AGV movement
```

------------------------------------------------------------------------

# 23. Current Status Summary

As of the current development stage:

The camera diagnostics below are historical notes from an earlier bring-up
attempt. Recheck them on the current Jetson before treating them as the current
hardware state. The repository contains the unified control code, but this
README does not record a successful end-to-end hardware run.

  -----------------------------------------------------------------------
  Component                           Status
  ----------------------------------- -----------------------------------
  Jetson Nano                         Working

  Project directory                   Created

  Camera device `/dev/video0`         Detected

  IMX219 identification               Detected

  CSI camera driver                   Loaded/visible

  Argus capture                       **Failing**

  OpenCV camera capture               **Failing because camera frames are
                                      unavailable**

  Yellow-line concept                 Finalized

  Yellow-line script                  Created

  Yellow-line live test               Hardware validation not recorded here

  Unified camera/LiDAR/STM32 code      Present in `agv_main.py`; hardware
                                      validation not established by this doc

  Web dashboard                       Planned; see `WEB_DASHBOARD_PLAN.md`

  Human detection                     Planned

  Full autonomous AGV                 Pending
  -----------------------------------------------------------------------

------------------------------------------------------------------------

# 24. Current Blocker

The immediate blocker is:

``` text
IMX219 I²C communication failure
```

Evidence:

``` text
no acknowledge from address 0x10
Error writing mode
calibration failed with -121
Failed to create CaptureSession
```

Therefore the next engineering task is:

> **Restore reliable IMX219 CSI camera capture on the Jetson Nano.**

After successful camera capture:

``` text
camera_test.py
      ↓
yellow_line.py
      ↓
line position
      ↓
direction
      ↓
motor control
```

------------------------------------------------------------------------

# 25. Recommended Development Order

Do not integrate all subsystems simultaneously.

Use this order:

``` text
1. Camera
      ↓
2. Yellow-line detection
      ↓
3. Line position
      ↓
4. Direction decision
      ↓
5. Human detection
      ↓
6. LiDAR
      ↓
7. Motor control
      ↓
8. Physical AGV
      ↓
9. Sensor fusion
      ↓
10. Final autonomous demonstration
```

This minimizes debugging complexity.

If the vehicle fails, each layer can be tested independently.

------------------------------------------------------------------------

# 26. Project Vision

The long-term system is intended to become:

``` text
                         AGV
                          │
             ┌────────────┴────────────┐
             │                         │
          CAMERA                     LiDAR
             │                         │
             ▼                         ▼
       Computer Vision            Distance Data
             │                         │
       ┌─────┴─────┐             ┌─────┴─────┐
       │           │             │           │
     Line       Human         Obstacle     Distance
   Detection   Detection       Detection    Check
       │           │             │           │
       └───────────┴─────────────┴───────────┘
                         │
                         ▼
                  Decision Layer
                         │
             ┌───────────┴───────────┐
             │                       │
        Navigation               Safety
             │                       │
             └───────────┬───────────┘
                         │
                         ▼
                   Motor Control
                         │
                         ▼
                       AGV
```

The project is therefore progressing from a **computer-vision
prototype** toward a **complete autonomous mobile robot**.

------------------------------------------------------------------------

# 27. Conclusion

The AGV project has completed the initial platform and architecture
decisions and has entered the computer-vision implementation phase.

The Jetson Nano is established as the primary processing platform, the
ESP32-CAM dependency has been removed from the current CV workflow, and
the project has selected a yellow navigation line on a gray floor for
the initial line-following system.

The current technical blocker is the IMX219 CSI camera's failure to
communicate correctly over I²C during sensor initialization. The camera
is visible as `/dev/video0`, but the Jetson cannot establish a working
Argus capture session.

Once camera capture is restored, development can continue directly with
the existing `camera_test.py` and `yellow_line.py` programs.

The immediate milestone is therefore:

> **Get stable live IMX219 frames → validate yellow-line detection →
> calculate line position → generate steering direction.**

From there, the project can progress systematically toward human
detection, LiDAR integration, motor control, and the final autonomous
AGV demonstration.
