#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
import time

class PS4MotorControl(Node):

    def __init__(self):
        super().__init__('r2_ps4_node')
        self.subscription = self.create_subscription(
            Joy,
            'ps4',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Relay setup
        self.relay_pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)  # Ensure the relay is initially OFF
        self.relay_state = False

        # Stepper motor setup
        self.step_pin = 18
        self.dir_pin = 27
        self.enable_pin = 22
        self.alarm_pin = 23
        self.speed_delay = 0.0001  # Speed control delay in seconds

        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.alarm_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Assuming alarm pin is active high

        GPIO.output(self.enable_pin, GPIO.LOW)  # Enable the motor

        self.last_angle_contest = None
        self.last_angle_test = None
        self.first_input_contest = True
        self.first_input_test = True
        self.in_progress = False

        # Button state
        self.prev_button_L1_state = 0
        self.prev_button_L2_state = 0
        self.prev_button_x_state = 0
        self.prev_button_circle_state = 0
        self.prev_button_tri_state = 0
        

    def listener_callback(self, msg):
        # Extract the state of buttons
        button_x = msg.buttons[0]
        button_circle = msg.buttons[1]
        button_tri = msg.buttons[2]
        button_L1 = msg.buttons[4]
        button_L2 = msg.buttons[6]

        # Handle relay toggle with button_circle (button 1)
        if button_circle == 1 and self.prev_button_circle_state == 0:
            self.get_logger().info('Button 1 pressed')
            self.toggle_relay()
        elif button_circle == 0 and self.prev_button_circle_state == 1:
            self.get_logger().info('Button 1 released')

        self.prev_button_circle_state = button_circle
        
        if button_tri == 1 and self.prev_button_tri_state == 0 and not self.in_progress:
            self.get_logger().info('Button 2 pressed')
            if self.first_input_test or self.last_angle_test == -65:
                self.in_progress = True
                self.rotate_to_angle(65)
                self.last_angle_test = 65
                self.first_input_test = False
                self.in_progress = False

        if button_x == 1 and self.prev_button_x_state == 0 and not self.in_progress:
            self.get_logger().info('Button 0 pressed')
            if self.last_angle_test == 65:
                self.in_progress = True
                self.rotate_to_angle(-65)
                self.last_angle_test = -65
                self.in_progress = False
	
        # Handle motor control with button_L1 (button 4) for up and button_L2 (button 6) for down
        if button_L1 == 1 and self.prev_button_L1_state == 0 and not self.in_progress:
            self.get_logger().info('Button 4 pressed')
            if self.first_input_contest or self.last_angle_contest == -150:
                self.in_progress = True
                self.rotate_to_angle(150)
                self.last_angle_contest = 150
                self.first_input_contest = False
                self.in_progress = False

        if button_L2 == 1 and self.prev_button_L2_state == 0 and not self.in_progress:
            self.get_logger().info('Button 6 pressed')
            if self.last_angle_contest == 150:
                self.in_progress = True
                self.rotate_to_angle(-150)
                self.last_angle_contest = -150
                self.in_progress = False

        self.prev_button_L1_state = button_L1
        self.prev_button_L2_state = button_L2
        self.prev_button_tri_state = button_tri
        self.prev_button_x_state = button_x

    def toggle_relay(self):
        self.relay_state = not self.relay_state
        GPIO.output(self.relay_pin, GPIO.HIGH if self.relay_state else GPIO.LOW)
        state_str = "ON" if self.relay_state else "OFF"
        self.get_logger().info(f'Relay is {state_str}')

    def rotate_to_angle(self, angle):
        steps_per_revolution = 800  # Adjust for your motor
        steps = int((angle / 18.0) * steps_per_revolution)  # Convert angle to steps

        if steps > 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)  # Set direction clockwise
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)  # Set direction counterclockwise
            steps = -steps  # Make steps positive

        for _ in range(steps):
            if GPIO.input(self.alarm_pin) == GPIO.HIGH:
                self.get_logger().info("Alarm detected! Stopping.")
                return  # Stop further execution

            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.speed_delay)  # Speed control delay
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(self.speed_delay)  # Speed control delay

def main(args=None):
    rclpy.init(args=args)
    ps4_motor_control = PS4MotorControl()
    try:
        rclpy.spin(ps4_motor_control)
    except KeyboardInterrupt:
        pass
    finally:
        ps4_motor_control.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
