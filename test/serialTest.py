import serial
import time

# --- SERIAL COMMUNICATION SETUP ---
# IMPORTANT: Find your ESP32's COM port.
# Update 'COM7' to your actual port.
try:
    esp32 = serial.Serial(port="COM7", baudrate=115200, timeout=0.1)
    print("ESP32 Connected.")
except serial.SerialException as e:
    print(f"Error connecting to ESP32: {e}")
    print("Please check the COM port and ensure the ESP32 is connected.")
    exit()

# Give the connection a moment to settle
time.sleep(2)

# --- COORDINATE SEQUENCE ---
# Format: (X, Y)
# These values will be sent directly to the ESP32 for servo control.
# Remember, X and Y here are just arbitrary numbers for the test,
# the ESP32 will map them to servo angles based on its FRAME_WIDTH/HEIGHT
# and MIN/MAX_ANGLE settings.
coordinates_sequence = [
    (0, 0),         # Corresponds to servoX=MIN_ANGLE, servoY=MIN_ANGLE
    (90, 90),       # Corresponds to servoX=mid_angle, servoY=mid_angle
    (180, 180),     # Corresponds to servoX=MAX_ANGLE, servoY=MAX_ANGLE
    (90, 90),       # Back to middle
    (0, 0)          # Back to start
]

# Delay between sending each coordinate pair (in seconds)
SEND_DELAY = 1.0

def send_to_esp32(x, y):
    """Formats and sends coordinates to the ESP32."""
    data_to_send = f"{x},{y}\n"
    esp32.write(data_to_send.encode())
    print(f"Sent: {data_to_send.strip()}")


print("Starting coordinate sequence test...")

try:
    while True: # Loop indefinitely to repeat the sequence
        for x, y in coordinates_sequence:
            send_to_esp32(x, y)
            time.sleep(SEND_DELAY)
        time.sleep(0.5) # Small pause before repeating the whole sequence
except KeyboardInterrupt:
    print("\nSequence interrupted by user.")
finally:
    # Always try to close the serial port
    print("Closing serial connection.")
    esp32.close()

print("Script finished.")