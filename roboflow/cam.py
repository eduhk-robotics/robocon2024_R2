import cv2
from ultralytics import YOLO
import os
import time

def main():
    # Set the model path relative to the script location
    script_dir = os.path.dirname(__file__)
    model_path = os.path.join(script_dir, 'model', 'weights', 'best.pt')

    # Load the lighter YOLOv8 model (if available)
    model = YOLO(model_path)

    # Open the UVC camera
    cap = cv2.VideoCapture('/dev/video0')  # Use the correct device path for your UVC camera

    if not cap.isOpened():
        print("Error: Could not open UVC camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Resize the frame for faster processing (reduce the size)
        resized_frame = cv2.resize(frame, (320, 240))  # Resize to a smaller resolution

        # Perform inference on the frame
        results = model(resized_frame)

        # Annotate the frame
        annotated_frame = resized_frame.copy()
        for result in results:
            boxes = result.boxes  # Box objects
            for box in boxes:
                # Extract coordinates for each box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]  # Confidence
                cls = box.cls[0]  # Class
                label = model.names[int(cls)]  # Class label

                # Draw rectangle and label on the frame
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(annotated_frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the results
        cv2.imshow('YOLOv8 Inference', annotated_frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Wait for 1 second
        time.sleep(1)

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
