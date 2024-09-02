import cv2
import requests
import numpy as np

# Replace with your Roboflow API key
API_KEY = 'cIqv4vSsVjV3YnzD7lEX'
# Replace with your model's endpoint from Roboflow
MODEL_ENDPOINT = 'https://detect.roboflow.com/robocon-2024-training/2'

def get_predictions(frame, api_key, model_endpoint):
    # Encode the frame as a JPEG image
    _, img_encoded = cv2.imencode('.jpg', frame)
    img_bytes = img_encoded.tobytes()
    
    # Prepare headers and parameters for the request
    headers = {
        'Content-Type': 'application/x-www-form-urlencoded'
    }
    params = {
        'api_key': api_key
    }

    # Send the request to the Roboflow model
    response = requests.post(model_endpoint, params=params, headers=headers, data=img_bytes)

    # Check if the request was successful
    if response.status_code == 200:
        predictions = response.json()
        return predictions
    else:
        print(f"Failed to get predictions: {response.text}")
        return None

def draw_predictions(frame, predictions):
    for prediction in predictions['predictions']:
        x = prediction['x']
        y = prediction['y']
        width = prediction['width']
        height = prediction['height']
        class_name = prediction['class']
        confidence = prediction['confidence']

        # Draw a rectangle around the object
        start_point = (int(x - width / 2), int(y - height / 2))
        end_point = (int(x + width / 2), int(y + height / 2))
        color = (0, 255, 0)
        thickness = 2
        cv2.rectangle(frame, start_point, end_point, color, thickness)

        # Put the class name and confidence on the rectangle
        text = f"{class_name}: {confidence:.2f}"
        cv2.putText(frame, text, (int(x - width / 2), int(y - height / 2) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

def main():
    # Initialize webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame.")
            break
        
        # Get predictions for the current frame
        predictions = get_predictions(frame, API_KEY, MODEL_ENDPOINT)
        
        if predictions:
            # Draw the predictions on the frame
            draw_predictions(frame, predictions)
        
        # Display the frame with predictions
        cv2.imshow('Webcam - Object Detection', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
