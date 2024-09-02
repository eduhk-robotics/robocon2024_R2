import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotorControlSubscriber(Node):
    def __init__(self):
        super().__init__('motor_control_subscriber')
        self.subscription = self.create_subscription(
            String,
            'motor_control',
            self.motor_control_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port to match your setup

    def motor_control_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received control command: "%s"' % command)
        
        # Send the command to the Arduino
        self.ser.write((command + '\n').encode('utf-8'))
        self.get_logger().info('Sent to Arduino: "%s"' % command)
        
        # Ensure the command is actually sent
        time.sleep(0.01)  # Give some time for the command to be processed
        
        # Check for response from Arduino
        if self.ser.in_waiting > 0:
            response = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info('Arduino response: "%s"' % response)
        else:
            self.get_logger().info('No response from Arduino')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()