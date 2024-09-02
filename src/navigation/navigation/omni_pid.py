import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
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
        # Subscribe to driving commands
        self.subscription_driving = self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )
        # Subscribe to motor speed data from the motor_data topic
        self.subscription_motor = self.create_subscription(
            String,
            'motor_data',
            self.motor_data_callback,
            10
        )
        # Publisher for motor control
        self.publisher_ = self.create_publisher(String, 'motor_control', 10)

        # Initialize PID controllers for each motor
        self.pid_controllers = [PID(1.0, 0.051, 0.8) for _ in range(4)]
        self.motor_speeds = [0, 0, 0, 0]  # Motor speeds for motors 1 to 4
        self.driving_command = [0.0, 0.0, 0.0]  # [direction, plane_speed, rotation_speed]

    def motor_data_callback(self, msg):
        """Update motor speeds with data from the motor_data topic."""
        try:
            # Example format of motor_data: "1,2000;2,2500;3,1800;4,2100"
            motor_data_strings = msg.data.split(';')
            for motor_data_string in motor_data_strings:
                motor_info = motor_data_string.split(',')
                motor_id = int(motor_info[0]) - 1  # Convert 1-indexed ID to 0-indexed
                motor_speed = int(motor_info[1])
                if 0 <= motor_id < 4:
                    self.motor_speeds[motor_id] = motor_speed
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing motor data: {e}")

    def driving_callback(self, msg):
        """Handle driving command input and compute setpoints for each motor."""
        self.driving_command = msg.data
        direction = self.driving_command[0]
        plane_speed = self.driving_command[1] * 0.6
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
            vx - vy + rotation_speed,  # Motor 3
            -vx - vy + rotation_speed  # Motor 4
        ]

        # Update the setpoints for each motor PID controller
        for i in range(4):
            self.pid_controllers[i].setpoint = motor_speeds_setpoint[i]

        self.calculate_and_publish_motor_powers()

    def calculate_and_publish_motor_powers(self):
        """Calculate motor power outputs based on the current speeds and publish control commands."""
        motor_powers = []
        for i in range(4):
            power = self.pid_controllers[i].update(self.motor_speeds[i])
            motor_powers.append(int(power))

        # Prepare and publish control commands to the motors
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
