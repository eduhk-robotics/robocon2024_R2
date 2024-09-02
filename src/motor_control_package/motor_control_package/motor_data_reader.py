import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class MotorDataReader(Node):

    def __init__(self):
        super().__init__('motor_data_reader')
        self.publisher_ = self.create_publisher(String, 'motor_data', 10)
        # Updated serial port and baud rate
        self.ser = self.setup_serial('/dev/ttyACM0', 115200)

        if self.ser:
            # Start a thread to continuously read from the serial port
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()

    def setup_serial(self, port, baudrate):
        """Setup the serial connection to the Arduino."""
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial connection established on {port} at {baudrate} baud.")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to establish serial connection: {e}")
            return None

    def read_serial_data(self):
        """Continuously read data from the serial port and publish it."""
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                arduino_data = self.ser.readline().decode('utf-8').strip()
                #self.get_logger().info(f"Raw data from Arduino: {arduino_data}")  # Print raw data for debugging
                
                if arduino_data:  # Ensure we have valid data
                    formatted_data = self.format_motor_data(arduino_data)
                    if formatted_data:
                        self.publisher_.publish(String(data=formatted_data))
                        # self.get_logger().info(f'Published motor data: "{formatted_data}"')

    def format_motor_data(self, raw_data):
        """Format the motor data from the Arduino to the correct string format for the omni_pid node."""
        try:
            # self.get_logger().info(f"Raw data before formatting: {raw_data}")  # Debugging raw data
            motor_data_list = raw_data.split(';')  # Split by semicolon to get each motor data
            formatted_data_list = []

            for motor_data in motor_data_list:
                motor_info = motor_data.split(',')
                if len(motor_info) == 4:  # Ensure we have motor_id, speed, angle, and torque
                    motor_id = int(motor_info[0])
                    motor_speed = int(motor_info[1])
                    motor_angle = int(motor_info[2])
                    motor_torque = int(motor_info[3])
                    formatted_data_list.append(f"{motor_id},{motor_speed},{motor_angle},{motor_torque}")
                else:
                    self.get_logger().error("Invalid motor data format")

            formatted_data = ";".join(formatted_data_list)  # Rejoin into the final string format
            return formatted_data
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error formatting motor data: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    motor_data_reader = MotorDataReader()
    rclpy.spin(motor_data_reader)
    motor_data_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
