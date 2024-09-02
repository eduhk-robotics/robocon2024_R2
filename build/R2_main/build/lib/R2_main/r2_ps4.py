#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
import time

class R2_PS4(Node):

    def __init__(self):
        super().__init__('r2_ps4')
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
        self.relay_state = False

        # Button state
        self.prev_button_circle_state = 0

    def listener_callback(self, msg):
        # Extract the state of button [1]
        button_circle = msg.buttons[1]

        # Check if button [1] is pressed and released
        if button_circle == 1 and self.prev_button_circle_state == 0:
            self.get_logger().info('Button 1 pressed')
            self.toggle_relay()
        elif button_circle == 0 and self.prev_button_circle_state == 1:
            self.get_logger().info('Button 1 released')

        self.prev_button_circle_state = button_circle

    def toggle_relay(self):
        self.relay_state = not self.relay_state
        GPIO.output(self.relay_pin, GPIO.LOW if self.relay_state else GPIO.HIGH)
        state_str = "ON" if self.relay_state else "OFF"
        self.get_logger().info(f'Relay is {state_str}')

def main(args=None):
    rclpy.init(args=args)
    r2_ps4 = R2_PS4()
    try:
        rclpy.spin(r2_ps4)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
