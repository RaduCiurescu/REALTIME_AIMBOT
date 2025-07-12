import cv2
import numpy as np
import serial
import time
import turret_angles  # Our custom module



lower_blue = np.array([90, 60, 40])
upper_blue = np.array([130, 255, 255])

# ==============================================================================
# --- CALIBRATION & CONFIGURATION ---
# ==============================================================================

# --- Serial Communication ---
SERIAL_PORT = "COM7"  # <-- CHANGE THIS to your ESP32's COM port
BAUD_RATE = 115200  # <-- Use a faster baud rate for smoother tracking

# --- Camera Settings ---
FRAME_WIDTH = 640         # Lowe    r resolution is faster
FRAME_HEIGHT = 480
# CALCULATE CENTER ONCE

CAMERA_CENTER_X = FRAME_WIDTH / 2
CAMERA_CENTER_Y = FRAME_HEIGHT / 2
REAL_KNOWN_OBJECT_DIAMETER_CM = 23.0
#======================
TURRET_OFFS_X = -2  # cm (2cm to the left of turret's center)
TURRET_OFFS_Y = 2  # cm (2cm up from turret's center)
TURRET_OFFS_Z = 1  # cm (1cm in front of turret's center)
#======================
CALIB_A = 0.0855
CALIB_B = -13.33
CALIB_C = 621.4
#==========================
# --- Setup ---
try:
    esp32 = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    print(f"ESP32 Connected on {SERIAL_PORT}.")
except serial.SerialException as e:
    print(f"Error connecting to ESP32: {e}")
    exit()
time.sleep(2)  # Give ESP32 time to initialize after connection







#===========================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam. Check if it's connected and not in use.")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Check if frame size was actually set (some cameras ignore set properties)
if cap.get(cv2.CAP_PROP_FRAME_WIDTH) != FRAME_WIDTH or \
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT) != FRAME_HEIGHT:
    print(f"Warning: Camera did not set to {FRAME_WIDTH}x{FRAME_HEIGHT}.")
    print(f"Actual resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    # You might need to adjust CAMERA_CENTER_X/Y if actual resolution differs

while True:
    Z_real = 23
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    if not ret:
        print("Failed to grab frame.")
        break
        # Show the video feed with overlays
        # 1) convert to HSV, 2) threshold, 3) clean up
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours:
        # pick the largest contour
        c = max(contours, key=cv2.contourArea)

        # get its bounding rectangle
        x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(c)

        # compute the centroid from that rectangle
        cx = x_rect + w_rect // 2
        cy = y_rect + h_rect // 2

        # re-zero around image center
        centerOrigin_X = cx - FRAME_WIDTH / 2
        centerOrigin_Y = FRAME_HEIGHT / 2 - cy
        #distance_virtual = (((hand_x2 - hand_x1) ** 2) + ((hand_y2 - hand_y1) ** 2)) ** (1 / 2)
        # draw the rectangle and a dot at the center
        cv2.rectangle(
            frame,
            (x_rect, y_rect),
            (x_rect + w_rect, y_rect + h_rect),
            (255, 0, 0),
            2
        )
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # label it
        text = f"({centerOrigin_X:.1f}, {centerOrigin_Y:.1f})"
        cv2.putText(
            frame,
            text,
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        x_real=centerOrigin_X *(5/w_rect)
        y_real=centerOrigin_Y * (5/w_rect)
        angles= turret_angles.turret(x_real,y_real,25)
        angles.offsets(0,2,-1)
        angles.getAngles()
        data_to_send = f"{int(angles.getTheta_x())},{int(angles.getTheta_y()+1)}\n"
        esp32.write(data_to_send.encode())

        print(f"OBIECTUL E IN:{centerOrigin_X},{centerOrigin_Y}")
        print(f"x:{int(angles.getTheta_x())} si y:{int(angles.getTheta_y())}")

    cv2.imshow("3D Object Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()