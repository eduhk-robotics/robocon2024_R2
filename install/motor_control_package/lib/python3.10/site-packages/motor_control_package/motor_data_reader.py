import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorDataReader(Node):
    def __init__(self):
        super().__init__('motor_data_reader')
        self.publisher_ = self.create_publisher(String, 'motor_data', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port to match your setup
        self.timer = self.create_timer(0.02, self.timer_callback)  # Read data every 10ms
        self.motor_data = ["0,0,0,0,0,0,0,0"] * 8  # Initialize motor data

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            arduino_data = self.ser.readline().decode('utf-8').strip()
            if arduino_data:  # Ensure we have valid data
                self.update_motor_data(arduino_data)
                msg = String()
                msg.data = self.format_motor_data()
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing motor data: "%s"' % msg.data)

    def update_motor_data(self, arduino_data):
        try:
            data_parts = arduino_data.split(';')
            for part in data_parts:
                motor_info = part.split(',')
                if len(motor_info) == 4:
                    motor_id = int(motor_info[0]) - 1
                    if 0 <= motor_id < 8:
                        self.motor_data[motor_id] = part
        except ValueError as e:
            self.get_logger().error(f"Error parsing motor data: {e}")

    def format_motor_data(self):
        return ";".join(self.motor_data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDataReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()