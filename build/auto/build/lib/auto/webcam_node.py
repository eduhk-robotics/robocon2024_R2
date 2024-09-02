import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np

API_KEY = 'cIqv4vSsVjV3YnzD7lEX'
MODEL_ENDPOINT = 'https://detect.roboflow.com/robocon-2024-training/2'

class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.detection_publisher_ = self.create_publisher(String, 'detections', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        # Convert frame to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

        # Get predictions
        predictions = self.get_predictions(frame)
        if predictions:
            detection_str = self.format_predictions(predictions)
            self.detection_publisher_.publish(String(data=detection_str))
            self.draw_predictions(frame, predictions)

        # Display the frame with predictions
        cv2.imshow('Webcam - Object Detection', frame)
        cv2.waitKey(1)

    def get_predictions(self, frame):
        _, img_encoded = cv2.imencode('.jpg', frame)
        img_bytes = img_encoded.tobytes()

        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        params = {'api_key': API_KEY}

        response = requests.post(MODEL_ENDPOINT, params=params, headers=headers, data=img_bytes)
        if response.status_code == 200:
            return response.json()
        else:
            self.get_logger().error(f"Failed to get predictions: {response.text}")
            return None

    def format_predictions(self, predictions):
        return '\n'.join([f"{pred['class']}: {pred['confidence']:.2f}" for pred in predictions['predictions']])

    def draw_predictions(self, frame, predictions):
        for prediction in predictions['predictions']:
            x = prediction['x']
            y = prediction['y']
            width = prediction['width']
            height = prediction['height']
            class_name = prediction['class']
            confidence = prediction['confidence']

            start_point = (int(x - width / 2), int(y - height / 2))
            end_point = (int(x + width / 2), int(y + height / 2))
            color = (0, 255, 0)
            thickness = 2
            cv2.rectangle(frame, start_point, end_point, color, thickness)

            text = f"{class_name}: {confidence:.2f}"
            cv2.putText(frame, text, (int(x - width / 2), int(y - height / 2) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
