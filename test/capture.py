import cv2
import numpy as np

def start_camera_feed():
    cap = cv2.VideoCapture(0)  # 0 indicates the default camera

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    while True:
        ret, frame = cap.read() # Read a frame from the camera

        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the captured frame (for testing)
        cv2.imshow('Camera Feed', frame)

        # Press 'q' to quit the camera feed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release() # Release the camera
    cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == "__main__":
    start_camera_feed()