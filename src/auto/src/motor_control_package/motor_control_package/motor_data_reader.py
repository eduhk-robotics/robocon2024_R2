import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorDataReader(Node):
    def __init__(self):
        super().__init__('motor_data_reader')
        self.publisher_ = self.create_publisher(String, 'motor_data', 10)
        self.ser = self.setup_serial('/dev/ttyACM0', 115200)
        self.timer = self.create_timer(0.02, self.timer_callback)  # Read data every 20ms
        self.motor_data = ["0,0"] * 8  # Initialize motor data for 8 motors

    def setup_serial(self, port, baudrate):
        """Setup the serial connection to the Arduino."""
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial connection established on {port} at {baudrate} baud.")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to establish serial connection: {e}")
            return None

    def timer_callback(self):
        """Callback function that reads data from the serial port and publishes it."""
        if self.ser and self.ser.in_waiting > 0:
            arduino_data = self.ser.readline().decode('utf-8').strip()
            if arduino_data:  # Ensure we have valid data
                self.update_motor_data(arduino_data)
                self.publish_motor_data()

    def update_motor_data(self, arduino_data):
        """Update the motor data with new values from Arduino."""
        try:
            data_parts = arduino_data.split(';')
            for part in data_parts:
                motor_info = part.split(',')
                if len(motor_info) == 2:  # Only expect motor ID and speed
                    motor_id = int(motor_info[0]) - 1
                    if 0 <= motor_id < 8:
                        self.motor_data[motor_id] = part
        except ValueError as e:
            self.get_logger().error(f"Error parsing motor data: {e}")

    def format_motor_data(self):
        """Format the motor data into a string suitable for publishing."""
        return ";".join(self.motor_data)

    def publish_motor_data(self):
        """Publish the motor data."""
        msg = String()
        msg.data = self.format_motor_data()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing motor data: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MotorDataReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
