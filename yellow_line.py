import cv2
import numpy as np

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, "
    "format=NV12, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Could not open IMX219")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("ERROR: Could not read camera")
        break

    # Convert image to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow HSV range
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    # Create yellow mask
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Remove small noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    output = frame.copy()

    if contours:
        # Largest yellow object
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if area > 500:

            M = cv2.moments(contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Camera center
                center_x = frame.shape[1] // 2

                # Draw detected line
                cv2.drawContours(output, [contour], -1, (0, 255, 0), 3)
                cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)

                # Camera center
                cv2.line(
                    output,
                    (center_x, 0),
                    (center_x, frame.shape[0]),
                    (255, 0, 0),
                    2
                )

                # Determine position
                error = cx - center_x

                if error < -80:
                    direction = "LEFT"
                elif error > 80:
                    direction = "RIGHT"
                else:
                    direction = "CENTER"

                cv2.putText(
                    output,
                    "YELLOW LINE DETECTED",
                    (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    output,
                    "Direction: " + direction,
                    (30, 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    output,
                    "Error: " + str(error),
                    (30, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2
                )

    else:
        cv2.putText(
            output,
            "NO YELLOW LINE",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    cv2.imshow("AGV Yellow Line", output)
    cv2.imshow("Yellow Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
