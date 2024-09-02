import cv2
from ultralytics import YOLO
import os
import threading
import time

global_annotated_frame = None
result_lock = threading.Lock()

class InferenceThread(threading.Thread):
    def __init__(self, model, frame):
        super().__init__()
        self.model = model
        self.frame = frame

    def run(self):
        global global_annotated_frame

        # Perform inference on the frame
        results = self.model(self.frame)
        
        # Annotate the frame
        annotated_frame = self.frame.copy()
        if results:
            for result in results:
                if result.boxes:
                    for box in result.boxes:
                        # Extract coordinates for each box
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = box.conf[0]  # Confidence
                        cls = box.cls[0]  # Class
                        label = self.model.names[int(cls)]  # Class label

                        # Draw rectangle and label on the frame
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(annotated_frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Safely update the global variable
        with result_lock:
            global_annotated_frame = annotated_frame

def main():
    # Get the current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct the path to the trained model
    model_path = os.path.join(script_dir, 'model', 'weights', 'best.pt')

    # Load the trained YOLOv8 model
    model = YOLO(model_path)

    # Open the UVC camera
    cap = cv2.VideoCapture('/dev/video0')  # Use the correct device path for your UVC camera

    if not cap.isOpened():
        print("Error: Could not open UVC camera.")
        return

    inference_thread = None
    last_time = time.time()

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Could not read frame. Trying again...")
                continue

            current_time = time.time()
            if (current_time - last_time >= 1):  # Check if at least 1 second has passed
                if inference_thread is None or not inference_thread.is_alive():
                    # Start a new inference thread if the previous one is done
                    inference_thread = InferenceThread(model, frame)
                    inference_thread.start()
                last_time = current_time

            # Display the current frame
            cv2.imshow('YOLOv8 Inference', frame)

            # Safely retrieve and display the annotated frame if available
            with result_lock:
                if global_annotated_frame is not None:
                    cv2.imshow('YOLOv8 Annotated', global_annotated_frame)

            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Wait for the last inference thread to finish
        if inference_thread:
            inference_thread.join()

    finally:
        # Release the webcam and close windows
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
