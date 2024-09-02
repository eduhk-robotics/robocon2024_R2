#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import math

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, deadband=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.deadband = deadband

    def compute(self, measurement, dt):
        error = self.setpoint - measurement

        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

class OmniPID(Node):

    def __init__(self):
        super().__init__('omni_pid')
        self.subscription_navigation = self.create_subscription(Float32MultiArray, 'driving', self.navigation_callback, 10)
        self.subscription_motor_speed = self.create_subscription(String, 'motor_data', self.motor_speed_callback, 10)
        self.publisher_motor_control = self.create_publisher(String, 'motor_control', 10)
        self.navigation_data = [0.0, 0.0, 0.0]  # [direction, plane_speed, rotation_speed]
        self.motor_speeds = [0, 0, 0, 0]  # Motor speeds for motors 1 to 4
        self.pid_controllers = [PIDController(0.3, 0.0, 0.00) for _ in range(4)]  # Adjusted PID controllers for each motor
        self.last_time = self.get_clock().now()

    def navigation_callback(self, msg):
        self.navigation_data = msg.data

    def motor_speed_callback(self, msg):
        motor_data_strings = msg.data.split(';')
        for motor_data_string in motor_data_strings:
            data = motor_data_string.split(',')
            try:
                can_id = int(data[0])
                motor_speed = int(data[1])
                if 1 <= can_id <= 4:
                    self.motor_speeds[can_id - 1] = motor_speed
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Failed to process motor data: {e}')
        
        self.control_motors()

    def control_motors(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        direction = self.navigation_data[0] * math.pi / 180  # Convert to radians
        plane_speed = self.navigation_data[1]
        rotation_speed = self.navigation_data[2]

        # Calculate target speeds for each motor
        target_speeds = [
            plane_speed * math.sin(direction + math.pi / 4) + rotation_speed,  # Motor 1
            plane_speed * math.cos(direction + math.pi / 4) + rotation_speed,  # Motor 2
            plane_speed * math.sin(direction - math.pi / 4) + rotation_speed,  # Motor 3
            plane_speed * math.cos(direction - math.pi / 4) + rotation_speed   # Motor 4
        ]

        # Calculate control signals using PID controllers
        control_signals = []
        for i in range(4):
            self.pid_controllers[i].setpoint = target_speeds[i]
            control_signal = self.pid_controllers[i].compute(self.motor_speeds[i], dt)
            control_signals.append(int(control_signal))

        # Prepare and publish the control messages
        commands = ";".join(f"{i+1},{control_signals[i]}" for i in range(4))
        msg = String()
        msg.data = commands
        self.publisher_motor_control.publish(msg)
        self.get_logger().info(f'Published control commands: {commands}')

def main(args=None):
    rclpy.init(args=args)
    omni_pid = OmniPID()
    rclpy.spin(omni_pid)
    omni_pid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
