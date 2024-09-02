#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class PS4Publisher(Node):

    def __init__(self):
        super().__init__('ps4_publisher')
        self.publisher_ = self.create_publisher(Joy, 'ps4', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def timer_callback(self):
        pygame.event.pump()
        msg = Joy()
        msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ps4_publisher = PS4Publisher()
    rclpy.spin(ps4_publisher)
    ps4_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
