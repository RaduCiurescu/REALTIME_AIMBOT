import cv2
import numpy as np
import serial
import time
from RUN import turret_angles

lower_blue = np.array([90, 60, 40])
upper_blue = np.array([130, 255, 255])

lower_light_green = np.array([90, 60, 40])
upper_light_green = np.array([130, 255, 255])

# ==============================================================================
# --- CALIBRATION & CONFIGURATION ---
# ==============================================================================

# --- Serial Communication ---
SERIAL_PORT = "COM6"  # <-- CHANGE THIS to your ESP32's COM port
BAUD_RATE = 115200  # <-- Use a faster baud rate for smoother tracking

# --- Camera Settings ---
FRAME_WIDTH = 640         # Lowe    r resolution is faster
FRAME_HEIGHT = 480
# CALCULATE CENTER ONCE

CAMERA_CENTER_X = FRAME_WIDTH / 2
CAMERA_CENTER_Y = FRAME_HEIGHT / 2
REAL_KNOWN_OBJECT_DIAMETER_CM = 12.0
#======================
TURRET_OFFS_X = -2  # cm (2cm to the left of turret's center)
TURRET_OFFS_Y = 2  # cm (2cm up from turret's center)
TURRET_OFFS_Z = 1  # cm (1cm in front of turret's center)
#======================
#20, 25, 30, 35, 40, 45, 50, 55, 60, 65

#300, 255, 190, 170, 148, 125, 109, 89, 82, 73


CALIB_A = 0.0008303358447026334
CALIB_B= -0.49283253237194863
CALIB_C = 94.41482659056284

#==========================
# --- Setup ---
try:
    esp32 = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    print(f"ESP32 Connected on {SERIAL_PORT}.")
except serial.SerialException as e:
    print(f"Error connecting to ESP32: {e}")
    #exit()
time.sleep(2)  # Give ESP32 time to initialize after connection

#====functions==

def calculate_z_real(pixel_radius):
    """
    Calculates real-world Z distance (depth) based on pixel radius.
    This version uses the estimated coefficients for a 23cm object.
    CRITICAL: YOU MUST CALIBRATE     THIS FOR YOUR SPECIFIC CAMERA/OBJECT.
    """
    if pixel_radius <= 0:
        return 9999.0  # Object too small or not found

    # New thresholds and equations estimated for a 23cm object
    # These are points where the linear/quadratic model shifts.
    # You will likely refine these thresholds after your own calibration.
    if pixel_radius >= 255:  # e.g., object is very close, pixel_radius > 110
        Z_real = (-0.1111 * pixel_radius) + 50.333  # Example: Z_real decreases as pixel_radius increases
    elif 170 <= pixel_radius < 255:  # e.g., mid-range, pixel_radius between 65 and 110
        Z_real = (-0.1176470588*pixel_radius) + 58.2608695652
    else:  # e.g., far range, pixel_radius < 65
        Z_real = (CALIB_A * (pixel_radius ** 2)) + (CALIB_B * pixel_radius) + CALIB_C

    return max(1.0, Z_real)  # Ensure Z is never zero or negative, minimum 1cm




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
    #this needs to be calculated
    Z_real = 23
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    if not ret:
        print("Failed to grab frame.")
        break
        # Show the video feed with overlays
        # 1) convert to HSV, 2) threshold, 3) clean up
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_light_green, upper_light_green)
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
        print(f"x:{int(x_rect)} si w:{int(w_rect)} si y:{int(y_rect)} si h:{int(h_rect)}")
        #distance_virtual = (((x_rect - w_rect) ** 2) + ((y_rect - h_rect) ** 2)) ** (1 / 2)
        distance_virtual = w_rect
        Z_real=calculate_z_real(w_rect) #radius of the object
        # draw the rectangle and a dot at the center
        cv2.rectangle(
            frame,
            (x_rect, y_rect),
            (x_rect + w_rect, y_rect + h_rect),
            (255, 0, 0),

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

        x_real=centerOrigin_X *(REAL_KNOWN_OBJECT_DIAMETER_CM/(w_rect/2))
        y_real=centerOrigin_Y * (REAL_KNOWN_OBJECT_DIAMETER_CM/(w_rect/2))
        #obj distance to ?
        angles= turret_angles.turret(x_real, y_real, Z_real)
        angles.offsets(TURRET_OFFS_X,TURRET_OFFS_Y,TURRET_OFFS_Z)
        angles.getAngles()
        data_to_send = f"{int(angles.getTheta_x())},{int(angles.getTheta_y())-15}\n"
        esp32.write(data_to_send.encode())

        print(f"OBIECTUL E IN:{centerOrigin_X},{centerOrigin_Y}=====: distanta de {Z_real}")
        print(f"x:{int(angles.getTheta_x())} si y:{int(angles.getTheta_y())}")

    cv2.imshow("3D Object Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()