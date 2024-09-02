import cv2
from ultralytics import YOLO
import os
import threading

# Global variables to store the annotated frame and detected objects
global_annotated_frame = None
global_detected_objects = []
result_lock = threading.Lock()

class InferenceThread(threading.Thread):
    def __init__(self, model, frame):
        super().__init__()
        self.model = model
        self.frame = frame

    def run(self):
        global global_annotated_frame, global_detected_objects  # Referencing the global variables

        # Perform inference on the frame
        results = self.model(self.frame)
        
        # Annotate the frame and collect detected objects
        annotated_frame = self.frame.copy()
        detected_objects = []
        if results:
            for result in results:
                if result.boxes:
                    for box in result.boxes:
                        # Extract coordinates for each box
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = box.conf[0]  # Confidence
                        cls = box.cls[0]  # Class
                        label = self.model.names[int(cls)]  # Class label

                        # Calculate the center point
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        detected_objects.append([label, center_x, center_y])

                        # Draw rectangle and label on the frame
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(annotated_frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        with result_lock:
            global_annotated_frame = annotated_frame  # Update the global variable
            global_detected_objects = detected_objects

def main():
    global global_annotated_frame, global_detected_objects

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
    previous_detected_objects = []

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Could not read frame. Trying again...")
                continue

            if inference_thread is None or not inference_thread.is_alive():
                # Start a new inference thread if the previous one is done
                inference_thread = InferenceThread(model, frame)
                inference_thread.start()

            # Display the annotated frame from the global variable
            with result_lock:
                if global_annotated_frame is not None:
                    cv2.imshow('YOLOv8 Inference', global_annotated_frame)
                    if global_detected_objects != previous_detected_objects:
                        print(global_detected_objects)  # Print the detected objects and their positions
                        previous_detected_objects = global_detected_objects.copy()

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
