import rclpy
from rclpy.node import Node
import serial

class MotorDataReader(Node):

    def __init__(self):
        super().__init__('motor_data_reader')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            arduino_data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            self.get_logger().info(f"Arduino data: {arduino_data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    motor_data_reader = MotorDataReader()
    rclpy.spin(motor_data_reader)
    motor_data_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
