
import cv2

def test_camera(index):
    cap = cv2.VideoCapture(index)

    if not cap.isOpened():
        print(f"Error: Could not open camera with index {index}")
        return False

    ret, frame = cap.read()
    if not ret:
        print(f"Error: Could not read frame from camera with index {index}")
        cap.release()
        return False

    print(f"Camera with index {index} is working")
    cap.release()
    return True

for i in range(5):  # Try the first 5 indices
    test_camera(i)
