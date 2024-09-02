import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

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
        commands = msg.data
        # self.get_logger().info(f'Received control commands: "{commands}"')
        
        try:
            # Send all commands to the Arduino at once
            self.ser.write((commands + '\n').encode('utf-8'))
            # self.get_logger().info(f'Sent to Arduino: "{commands}"')

            # # Check for response from Arduino
            # response = self.read_serial_response()
            # if response:
            #     self.get_logger().info(f'Arduino response: "{response}"')
            # else:
            #     self.get_logger().info('No response from Arduino')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send commands to Arduino: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

    def read_serial_response(self):
        response = ''
        while self.ser.in_waiting > 0:
            response += self.ser.readline().decode('utf-8').strip()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
