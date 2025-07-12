import cv2
from ultralytics import YOLO

def yolov8_object_detection():
    # Load a pre-trained YOLOv8 model
    # 'yolov8n.pt' is the nano model, good for quick testing and speed
    # You can also use yolov8s.pt (small), yolov8m.pt (medium), etc.
    model = YOLO('yolov8n.pt')

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Run YOLOv8 inference on the frame
        # The 'stream=True' argument makes it suitable for video streams
        results = model(frame, stream=True)

        # Iterate over results for each frame
        for r in results:
            # Get bounding boxes, classes, and confidence scores
            boxes = r.boxes

            for box in boxes:
                # Get box coordinates (top-left x, y, width, height)
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # x1, y1, x2, y2 are pixel coords

                # Get confidence score
                confidence = round(float(box.conf[0]), 2)

                # Get class ID and name
                class_id = int(box.cls[0])
                class_name = model.names[class_id]

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2) # Blue box

                # Calculate centroid of the bounding box
                cX = int((x1 + x2) / 2)
                cY = int((y1 + y2) / 2)

                # Put label with class name, confidence, and centroid
                label = f"{class_name} {confidence}: ({cX}, {cY})"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # White text

                # Draw a small circle at the centroid
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1) # Red circle


        # Display the frame with detections
        cv2.imshow('YOLOv8 Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    yolov8_object_detection()