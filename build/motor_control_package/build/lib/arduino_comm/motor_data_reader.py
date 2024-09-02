import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorDataReader(Node):
    def __init__(self):
        super().__init__('motor_data_reader')
        self.publisher_ = self.create_publisher(String, 'motor_data', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port to match your setup
        self.timer = self.create_timer(0.01, self.timer_callback)  # Read data every second

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            arduino_data = self.ser.readline().decode('utf-8').strip()
            msg = String()
            msg.data = arduino_data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing motor data: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDataReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()