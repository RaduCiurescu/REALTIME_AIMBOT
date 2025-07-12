import cv2
import numpy as np

# HSV color ranges (as defined above)
color_ranges = {
    # "red": {
    #     "lower": np.array([0, 100, 100]),
    #     "upper": np.array([10, 255, 255]),
    #     "lower2": np.array([170, 100, 100]),
    #     "upper2": np.array([180, 255, 255])
    # },
    # "green": {
    #     "lower": np.array([40, 40, 40]),
    #     "upper": np.array([80, 255, 255])
    # },
    "blue": {
        "lower": np.array([100, 50, 50]),
        "upper": np.array([140, 255, 255])
     }
    #,
    # "black": {
    #     "lower": np.array([0, 0, 0]),
    #     "upper": np.array([180, 255, 30])
    # },
    # "white": {
    #     "lower": np.array([0, 0, 180]),
    #     "upper": np.array([180, 30, 255])
    # }
}

def detect_colors_and_label():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Convert the frame from BGR to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_name, hsv_range in color_ranges.items():
            mask = None
            if color_name == "red":
                # Handle red's two ranges
                mask1 = cv2.inRange(hsv_frame, hsv_range["lower"], hsv_range["upper"])
                mask2 = cv2.inRange(hsv_frame, hsv_range["lower2"], hsv_range["upper2"])
                mask = cv2.add(mask1, mask2)
            else:
                # Create a mask for the current color
                mask = cv2.inRange(hsv_frame, hsv_range["lower"], hsv_range["upper"])

            # Optional: Apply morphological operations to clean up the mask
            # This helps remove noise and fill small gaps
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)


            # Find contours in the mask
            # cv2.RETR_EXTERNAL retrieves only outer contours
            # cv2.CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Filter out small contours (noise)
                area = cv2.contourArea(contour)
                if area > 500:  # Adjust this threshold as needed
                    # Get the bounding rectangle for the contour
                    x, y, w, h = cv2.boundingRect(contour)

                    # Draw the bounding rectangle
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) # Green rectangle

                    # Calculate the center of the label coordinates (centroid)
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        # Put text (color name and coordinates) on the frame
                        label = f"{color_name}: ({cX}, {cY})"
                        cv2.putText(frame, label, (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # White text

                        # Draw a small circle at the centroid
                        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1) # Red circle

        # Display the resulting frame
        cv2.imshow('Color Recognition', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_colors_and_label()