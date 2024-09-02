import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import threading
import math

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._prev_error = 0
        self._integral = 0

    def update(self, measured_value):
        error = self.setpoint - measured_value
        self._integral += error
        derivative = error - self._prev_error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error
        return output

class OmniPID(Node):
    def __init__(self):
        super().__init__('omni_pid')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'motor_control', 10)

        # Initialize PID controllers for each motor
        self.pid_controllers = [PID(0.9, 0.0,0.3) for _ in range(4)]
        #self.pid_controllers = [PID(0.9, 0.015,0.3) for _ in range(4)]
        #self.pid_controllers = [PID(0.27, 0.24,0.7) for _ in range(4)]
        self.motor_speeds = [0, 0, 0, 0]
        self.driving_command = [0.0, 0.0, 0.0]  # direction, plane_speed, rotation_speed

        # Set up serial connection to Arduino
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
        """Continuously read data from the serial port."""
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                arduino_data = self.ser.readline().decode('utf-8').strip()
                if arduino_data:  # Ensure we have valid data
                    self.update_motor_speeds(arduino_data)

    def update_motor_speeds(self, arduino_data):
        """Update the motor speeds with new values from Arduino."""
        try:
            data_parts = arduino_data.split(';')
            for part in data_parts:
                motor_info = part.split(',')
                if len(motor_info) == 2:  # Only expect motor ID and speed
                    motor_id = int(motor_info[0]) - 1
                    if 0 <= motor_id < 4:
                        self.motor_speeds[motor_id] = int(motor_info[1])  # Update motor speed
        except ValueError as e:
            self.get_logger().error(f"Error parsing motor data: {e}")

    def driving_callback(self, msg):
        self.driving_command = msg.data
        direction = self.driving_command[0]
        plane_speed = self.driving_command[1] * 0.5
        rotation_speed = self.driving_command[2] * 0.2

        # Convert direction to radians
        direction_rad = math.radians(direction)

        # Calculate the velocity components
        vx = plane_speed * math.cos(direction_rad)
        vy = plane_speed * math.sin(direction_rad)

        # Calculate each motor's speed using inverse kinematics for omni wheels
        motor_speeds_setpoint = [
            -vx + vy + rotation_speed,  # Motor 1
            vx + vy + rotation_speed,  # Motor 2
            vx - vy + rotation_speed, # Motor 3
            -vx - vy + rotation_speed  # Motor 4
        ]
	
        self.get_logger().info(f'motor_speeds_setpoint: "{motor_speeds_setpoint}"')
        # Update the setpoints for each motor PID controller
        for i in range(4):
            self.pid_controllers[i].setpoint = motor_speeds_setpoint[i]

        self.calculate_and_publish_motor_powers()

    def calculate_and_publish_motor_powers(self):
        motor_powers = []
        for i in range(4):
            power = self.pid_controllers[i].update(self.motor_speeds[i])
            motor_powers.append(int(power))
        

        self.get_logger().error(f"self.motor_speeds{self.motor_speeds}")
            
        #motor_powers[0] = -1 * motor_powers[0]
        #motor_powers[3] = -1 * motor_powers[3]
        

        commands = ";".join([f"{i+1},{power}" for i, power in enumerate(motor_powers)])
        self.publisher_.publish(String(data=commands))
        self.get_logger().info(f'Published motor powers: "{commands}"')

def main(args=None):
    rclpy.init(args=args)
    node = OmniPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
