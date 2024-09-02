#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from threading import Thread
import RPi.GPIO as GPIO
import random
import time

class R2Main(Node):

    def __init__(self):
        super().__init__('r2_main_node')
        
	self.relay_pin = 17
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(self.relay_pin, GPIO.OUT)

        self.X_distance = None
    	self.Y_distance = None
    	self.X_gyro = None
    	self.Y_gyro = None
    	self.Z_gyro = None
        
	self.silo_status = [0, 0, 0, 0, 0]
	self.silo_distances = [200, 955, 1710, 2465, 3220]

        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        
        self.sensor_subscriber = self.create_subscription(
            String,
            'sensor',
            self.sensor_callback,
            10)
        
    def sensor_callback(self, msg):
    	sensor_data = msg.data
    	sensor_data_list = sensor_data.spilt(";")
    	self.X_distance = sensor_data_list[1]
    	self.Y_distance = sensor_data_list[3]
    	self.X_gyro = sensor_data_list[5]
    	self.Y_gyro = sensor_data_list[6]
    	self.Z_gyro = sensor_data_list[7]	
    
    def go_forward(self, speed_percentage = 1):
    	self.direction = 0
    	self.plane_speed = 8192 * speed_percentage
    	self.rotation_speed = 0
    	send_driving_data(self.direction, self.plane_speed, self.rotation_speed)
    	
    def go_backward(self, speed_percentage = 1):
    	self.direction = 180
    	self.plane_speed = 8192 * speed_percentage
    	self.rotation_speed = 0
    	send_driving_data(self.direction, self.plane_speed, self.rotation_speed)
    
    def go_left(self, speed_percentage = 1):
    	self.direction = 270
    	self.plane_speed = 8192 * speed_percentage
    	self.rotation_speed = 0
    	send_driving_data(self.direction, self.plane_speed, self.rotation_speed)
    
    def go_right(self, speed_percentage = 1):
    	self.direction = 90
    	self.plane_speed = 8192 * speed_percentage
    	self.rotation_speed = 0
    	send_driving_data(self.direction, self.plane_speed, self.rotation_speed)
    
    def rotation(self, direction, speed_percentage=1):
	if direction == "L" or direction == "Left" or direction == "left":
	    self.rotation_speed = -8192
	elif if direction == "R" or direction == "Right" or direction == "right":
	    self.rotation_speed = 8192
	self.direction = 0
    	self.plane_speed = 0
    	self.rotation_speed = self.rotation_speed * speed_percentage
	send_driving_data(self.direction, self.plane_speed, self.rotation_speed)

    def stop(self):
    	self.direction = 0
    	self.plane_speed = 0
    	self.rotation_speed = 0
    	send_driving_data(self.directiondirection, self.plane_speed, self.rotation_speed)
    
    def send_driving_data(self, direction, plane_speed, rotation_speed):
        driving_msg = Float32MultiArray()
        driving_msg.data = [float(direction), float(plane_speed), float(rotation_speed)]
        
        self.publisher_.publish(driving_msg)
        self.get_logger().info(f"Direction={direction}, Plane Speed={plane_speed}, Rotation Speed={rotation_speed}")    
 
    def red_to_area3(self):
	while self.X_gyro < 30:
    	    go_forward()
	while self.X_gyro > 1:
	    go_forward()
    	while self.Y_distance > 240:
    	    go_forward()
    	stop()
    	time.sleep(1)
    	while self.X_distance < 3985:
    	    go_right()
    	stop()
    	time.sleep(1)
    	while self.X_gyro < 30:
    	    go_forward()
	while self.X_gyro > 1:
	    go_forward()
	while self.Y_distance > 1710:
    	    go_forward()
	while self.Z_gyro > -90:
	    rotation("R")

    def blue_to_area3(self):
	while self.X_gyro < 30:
    	    go_forward()
	while self.X_gyro > 1:
	    go_forward()
    	while self.Y_distance > 240:
    	    go_forward()
    	stop()
    	time.sleep(1)
    	while self.X_distance < 3985:
    	    go_left()
    	stop()
    	time.sleep(1)
    	while self.X_gyro < 30:
    	    go_forward()
	while self.X_gyro > 1:
	    go_forward()
	while self.Y_distance > 1710:
    	    go_forward()
	while self.Z_gyro < 90:
	    rotation("L")

    def ai_detection(self):
	pass

    def gripper_catch(self):
	GPIO.output(relay_pin, GPIO.HIGH)
	

    def gripper_release(self):
	GPIO.output(relay_pin, GPIO.LOW)

    def random_silo(self):
	while True:
	    chose_silo = random.randint(0, 4)
	    if self.silo_status[chose_silo] < 3:
		#self.silo_status[chose_silo] += 1
		return self.silo_distances[chose_silo]
	    


def main(args=None):
    rclpy.init(args=args)
    r2_main = R2Main()
    rclpy.spin(r2_main)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()