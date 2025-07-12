import cv2
import numpy as np
import serial
import time
import turret_angles  # Our custom module

# ==============================================================================
# --- CALIBRATION & CONFIGURATION ---
# ==============================================================================

# --- Serial Communication ---
SERIAL_PORT = "COM7"  # <-- CHANGE THIS to your ESP32's COM port
BAUD_RATE = 115200  # <-- Use a faster baud rate for smoother tracking

# --- Camera Settings ---
FRAME_WIDTH = 640  # Lower resolution is faster
FRAME_HEIGHT = 480
# CALCULATE CENTER ONCE
CAMERA_CENTER_X = FRAME_WIDTH / 2
CAMERA_CENTER_Y = FRAME_HEIGHT / 2

# --- Physical Object and Turret Offsets ---
# Set to your object's measured diameter (still 23.0 for your case)
REAL_KNOWN_OBJECT_DIAMETER_CM = 23.0

# YOUR PHYSICAL SETUP:
# Based on: Camera is 2cm LEFT, 2cm UP, and 1cm IN FRONT of the turret pivot.
# The coordinate system is from the turret's pivot point:
# +X is RIGHT, +Y is UP, +Z is FORWARD (towards the object).
TURRET_OFFS_X = -2  # cm (2cm to the left of turret's center)
TURRET_OFFS_Y = 2  # cm (2cm up from turret's center)
TURRET_OFFS_Z = 1  # cm (1cm in front of turret's center)

# --- Final Servo Angle Offsets ---
# These are the *base* angles. When turret_angles returns 0 for a target,
# the servo will go to this value.
# For 90 degrees as center:
SERVO_X_CENTER_ANGLE = 90
SERVO_Y_CENTER_ANGLE = 90

# These are *fine-tuning* offsets to correct for minor mechanical or
# software misalignments after initial calibration.
# Start these at 0 and adjust ONLY AFTER the main calibration steps.
SERVO_X_FINE_TUNE = 200
SERVO_Y_FINE_TUNE = 150

# --- Z-Axis (Depth) Calibration Data ---
# These are the ESTIMATED coefficients for a 23cm object.
# These will be the FIRST thing you need to RE-CALIBRATE ACCURATELY.
CALIB_A = 0.0855
CALIB_B = -13.33
CALIB_C = 621.4


def calculate_z_real(pixel_radius):
    """
    Calculates real-world Z distance (depth) based on pixel radius.
    This version uses the estimated coefficients for a 23cm object.
    CRITICAL: YOU MUST CALIBRATE THIS FOR YOUR SPECIFIC CAMERA/OBJECT.
    """
    if pixel_radius <= 0:
        return 9999.0  # Object too small or not found

    # New thresholds and equations estimated for a 23cm object
    # These are points where the linear/quadratic model shifts.
    # You will likely refine these thresholds after your own calibration.
    if pixel_radius >= 110:  # e.g., object is very close, pixel_radius > 110
        Z_real = (-0.28 * pixel_radius) + 105.8  # Example: Z_real decreases as pixel_radius increases
    elif 65 <= pixel_radius < 110:  # e.g., mid-range, pixel_radius between 65 and 110
        Z_real = (-1.16 * pixel_radius) + 199.6
    else:  # e.g., far range, pixel_radius < 65
        Z_real = (CALIB_A * (pixel_radius ** 2)) + (CALIB_B * pixel_radius) + CALIB_C

    return max(1.0, Z_real)  # Ensure Z is never zero or negative, minimum 1cm


# ==============================================================================
# --- MAIN PROGRAM LOGIC ---
# ==============================================================================

# --- Setup ---
try:
    esp32 = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
    print(f"ESP32 Connected on {SERIAL_PORT}.")
except serial.SerialException as e:
    print(f"Error connecting to ESP32: {e}")
    exit()
time.sleep(2)  # Give ESP32 time to initialize after connection

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

# Define the HSV range for the color blue
# These values are generic. You might need to fine-tune them for your lighting.
# Use an HSV color picker tool (like online ones or OpenCV's trackbar example)
# to find the ideal range for your specific blue object.
lower_blue = np.array([90, 60, 40])
upper_blue = np.array([130, 255, 255])


def send_to_esp32(x_angle, y_angle):
    """
    Formats and sends final servo angles to the ESP32.
    Clamps values to 0-180 to prevent out-of-range commands.
    """
    x_angle_clamped = np.clip(x_angle, 0, 180)  # Ensure angle is between 0 and 180
    y_angle_clamped = np.clip(y_angle, 0, 180)  # Ensure angle is between 0 and 180
    data_to_send = f"{int(x_angle_clamped)},{int(y_angle_clamped)}\n"
    try:
        esp32.write(data_to_send.encode())
        # print(f"Sent: {data_to_send.strip()}") # Uncomment for debugging serial output
    except serial.SerialTimeoutException:
        print("Warning: Serial write timed out.")
    except Exception as e:
        print(f"Error sending data over serial: {e}")


# --- Main Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Flip the frame horizontally for a mirror-like view (common for webcams)
    frame = cv2.flip(frame, 1)

    # Convert frame to HSV color space for better color filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask to isolate blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # Perform morphological operations to clean up the mask (remove small noise, close gaps)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask (find distinct blue blobs)
    contours, _ = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) > 0:
        # Find the largest contour by area and assume it's our target object
        c = max(contours, key=cv2.contourArea)
        # Get the minimum enclosing circle for the contour (center and radius)
        ((x_pixel, y_pixel), radius_pixel) = cv2.minEnclosingCircle(c)

        # Only process if the detected object is large enough (filter out tiny noise)
        if radius_pixel > 10:  # Minimum radius to consider a valid object
            # --- 1. Calculate Real-World Z (Depth) ---
            Z_real = calculate_z_real(radius_pixel)

            # --- 2. Calculate Real-World X and Y relative to CAMERA CENTER ---
            # X_real: Positive to the right of camera center, negative to the left.
            # (x_pixel - CAMERA_CENTER_X) gives pixel offset from center.
            # (REAL_KNOWN_OBJECT_DIAMETER_CM / (radius_pixel * 2)) is the cm-per-pixel scale.
            X_real = (x_pixel - CAMERA_CENTER_X) * \
                     (REAL_KNOWN_OBJECT_DIAMETER_CM / (radius_pixel * 2))

            # Y_real: Positive above camera center, negative below.
            # (CAMERA_CENTER_Y - y_pixel) inverts OpenCV's Y-axis (down is positive)
            # to standard Cartesian (up is positive).
            Y_real = (CAMERA_CENTER_Y - y_pixel) * \
                     (REAL_KNOWN_OBJECT_DIAMETER_CM / (radius_pixel * 2))

            # --- 3. Calculate Servo Angles ---
            # Pass the object's real-world coordinates relative to the camera to turret_angles.
            angles_calculator = turret_angles.turret(X_real, Y_real, Z_real)
            # Apply the camera's physical offset from the turret pivot.
            angles_calculator.offsets(
                offs_x=TURRET_OFFS_X,
                offs_y=TURRET_OFFS_Y,
                offs_z=TURRET_OFFS_Z,
            )
            angles_calculator.getAngles()  # Perform inverse kinematics internally

            # Get the calculated *relative* angles from the turret_angles module
            # These angles are how much to move FROM the turret's current neutral direction.
            # Then add the servo's physical center angle (e.g., 90 degrees) to get absolute servo values.
            final_servo_x = angles_calculator.getTheta_x() + SERVO_X_CENTER_ANGLE + SERVO_X_FINE_TUNE
            final_servo_y = angles_calculator.getTheta_y() + SERVO_Y_CENTER_ANGLE + SERVO_Y_FINE_TUNE

            # --- 4. Send Angles to ESP32 ---
            send_to_esp32(final_servo_x, final_servo_y)

            # --- 5. Display Information on Screen ---
            # Draw the detected circle and its center dot
            cv2.circle(frame, (int(x_pixel), int(y_pixel)), int(radius_pixel), (0, 255, 255), 2)
            cv2.circle(frame, (int(x_pixel), int(y_pixel)), 5, (0, 0, 255), -1)  # Center dot of object

            # Optionally draw a crosshair at the camera's assumed center for visual feedback
            cv2.line(frame, (int(CAMERA_CENTER_X), 0), (int(CAMERA_CENTER_X), FRAME_HEIGHT), (0, 255, 0), 1)
            cv2.line(frame, (0, int(CAMERA_CENTER_Y)), (FRAME_WIDTH, int(CAMERA_CENTER_Y)), (0, 255, 0), 1)

            # Display calculated real-world positions and servo angles
            label_pos = f"X:{X_real:.1f} Y:{Y_real:.1f} Z:{Z_real:.1f} cm"
            label_ang = f"Servo X:{int(final_servo_x)} Y:{int(final_servo_y)}"
            cv2.putText(
                frame,
                label_pos,
                (10, 30),  # Position for text (top-left is 0,0)
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                label_ang,
                (10, 60),  # Position for text
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            # Print to console for detailed debugging
            print(f"Pixel (X,Y,R): ({int(x_pixel)}, {int(y_pixel)}, {int(radius_pixel)})")
            print(f"Real (X,Y,Z): ({X_real:.1f}, {Y_real:.1f}, {Z_real:.1f}) cm")
            print(f"Calculated Servo (X,Y): ({final_servo_x:.1f}, {final_servo_y:.1f})")

    # Show the video feed with overlays
    cv2.imshow("3D Object Tracker", frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# --- Cleanup ---
print("Shutting down.")
send_to_esp32(90, 90)  # Center servos on exit, as a final command
time.sleep(0.1)  # Give a moment for the last command to send
cap.release()
cv2.destroyAllWindows()
if esp32.is_open:
    esp32.close()
    print("Serial port closed.")