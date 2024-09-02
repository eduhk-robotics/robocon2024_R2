import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import math

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class AngleRotation(Node):

    def __init__(self):
        super().__init__('angle_rotation')

        # PID controller for angle control for Motor ID 4
        self.pid = PID(0.08, 0.0, 0.0)  # Adjust the constants as necessary

        # Subscriber to the driving topic to get the direction
        self.subscription_driving = self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Subscriber to motor_data topic to get motor angles
        self.subscription_motor_data = self.create_subscription(
            String,
            'motor_data',
            self.motor_data_callback,
            10
        )

        # Publisher to send motor control commands
        self.publisher_ = self.create_publisher(String, 'motor_control', 10)

        self.target_angle = 0  # Setpoint for the PID controller
        self.motor_angle_4 = 0  # Store the current angle for Motor 4
        self.last_time = self.get_clock().now()

    def map_direction_to_angle(self, direction):
        """Maps a direction (0-360 degrees) to motor angles (0-8192)"""
        return int((direction / 360.0) * 8192)

    def driving_callback(self, msg):
        """Callback to handle new target direction from driving topic"""
        direction = msg.data[0]  # Extract direction (0-360 degrees)
        self.target_angle = self.map_direction_to_angle(direction)  # Map it to 0-8192 range

        # Update the PID setpoint to the target angle
        self.pid.setpoint = self.target_angle

        self.get_logger().info(f"Mapped angle: {self.target_angle}")

    def motor_data_callback(self, msg):
        """Callback to handle motor data and apply PID control for Motor ID 4 only"""
        motor_data_strings = msg.data.split(';')
        for motor_data_string in motor_data_strings:
            motor_info = motor_data_string.split(',')
            try:
                motor_id = int(motor_info[0])
                motor_angle = int(motor_info[2])  # Assuming motor_angle is at index 2
                if motor_id == 4:  # Only process Motor ID 4
                    self.motor_angle_4 = motor_angle  # Update the current angle of Motor 4
                    self.get_logger().error(f"Motor Angle: {motor_angle}")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error processing motor data: {e}")

        # Control Motor 4 to reach the target angle using PID
        self.control_motor_4()

    def control_motor_4(self):
        """Use PID to calculate motor control signal for Motor ID 4 and publish the command"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # The input to the PID is the current motor angle (motor_angle_4)
        pid_output = self.pid.update(self.motor_angle_4, dt)
        self.get_logger().info(f"Power: {pid_output}")
        # Prepare and publish control command for Motor 4
        command = f"4,{int(pid_output)}"
        control_msg = String()
        control_msg.data = command
        self.publisher_.publish(control_msg)

        # self.get_logger().info(f"Published control command for Motor 4: {command}")

def main(args=None):
    rclpy.init(args=args)
    angle_rotation = AngleRotation()
    rclpy.spin(angle_rotation)
    angle_rotation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
