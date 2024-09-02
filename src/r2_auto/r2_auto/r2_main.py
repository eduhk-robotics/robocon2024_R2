#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import RPi.GPIO as GPIO
import random
import time
import threading
import serial
import ast

class R2Main(Node):

    def __init__(self):
        super().__init__('r2_main_node')
        
        # Setup serial communication
        self.ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Open the serial port

        # Crash sensor setup
        CRASH_SENSOR_PIN = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CRASH_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(CRASH_SENSOR_PIN, GPIO.RISING, callback=self.crash_detected_callback)
        self.crash_to_start = False

        # Relay setup
        self.relay_pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT, initial=GPIO.LOW)
        self.relay_state = False

        # Stepper motor setup
        self.step_pin = 18
        self.dir_pin = 27
        self.enable_pin = 22
        self.alarm_pin = 23
        self.speed_delay = 0.0001

        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.alarm_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.last_angle = None
        self.first_input = True
        self.in_progress = False
        
        # Sensor variable setup
        self.sensor_data_list = []
        self.X_distance = self.Y_distance = self.X_gyro = self.Y_gyro = self.Z_gyro = self.Z_gyro_new = None
        self.silo_status = [0] * 3
        self.silo_distances = [955, 1710, 2465]

        # Publisher for driving data
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        
        # Subscription to sensor data
        self.sensor_subscriber = self.create_subscription(String, 'sensor', self.sensor_callback, 10)
        
        # Initialize the stepper motor to a starting position
        self.stepper_motor_init()

        # Use threading for concurrent execution
        self.sensor_data_received = threading.Event()
        
        # Create threads for different tasks
        self.subscription_thread = threading.Thread(target=self.spin_subscription)
        self.start_thread = threading.Thread(target=self.run_start)
        self.serial_thread = threading.Thread(target=self.read_serial_data)

        # Start the threads
        self.serial_thread.start()
        self.subscription_thread.start()
        self.start_thread.start()

    def crash_detected_callback(self, channel):
        self.crash_to_start = True


    def parse_data(self, data):
        """Parse the incoming serial data."""
        try:
            parsed_data = ast.literal_eval(data)
            # Ensure parsed data is a list
            if isinstance(parsed_data, list):
                return parsed_data
            else:
                return [parsed_data]
        except (ValueError, SyntaxError):
            self.get_logger().warning("Received data could not be parsed. Skipping...")
            return []  
        
    def read_serial_data(self):
        """Read data from the serial port continuously."""
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.readline().decode('utf-8').rstrip()
                    self.ai_ball_data = self.parse_data(data)  # [colour, X, Y, Area]
                    self.get_logger().info(f"Received AI Ball Data: {self.ai_ball_data}")
                except UnicodeDecodeError:
                    # Handle the decoding error
                    self.get_logger().warning("Received data could not be decoded. Skipping...")

    def spin_subscription(self):
        """Spin the ROS2 subscription thread."""
        rclpy.spin(self)

    def sensor_callback(self, msg):
        """Callback function for sensor data subscription."""
        self.get_logger().info('Received sensor data')
        self.sensor_data_list = msg.data.split(";")
        self.Y_distance = float(self.sensor_data_list[1])
        self.X_distance = float(self.sensor_data_list[3])
        self.X_gyro = float(self.sensor_data_list[5])
        self.Y_gyro = float(self.sensor_data_list[6])
        self.Z_gyro = float(self.sensor_data_list[7])
        self.Z_gyro_new = float(self.sensor_data_list[8])
        self.sensor_data_received.set()  # Signal that sensor data is received
       
    def run_start(self):
        """Wait for sensor data to be received and then start the process."""
        self.sensor_data_received.wait()  # Wait until sensor data is received
        while self.crash_to_start != True:
            self.get_logger().info("Wait for crash")
        # Trigger functions based on conditions, ensure you add conditions if needed
        #self.red_to_area2()
        #self.red_area2_to_area3()
        # Uncomment these if needed for the blue path
        self.blue_to_area2()
        self.blue_area2_to_area3()

    def send_driving_data(self, direction, plane_speed, rotation_speed):
        """Publish driving data to the 'driving' topic."""
        driving_msg = Float32MultiArray(data=[float(direction), float(plane_speed), float(rotation_speed)])
        self.publisher_.publish(driving_msg)
        self.get_logger().info(f"Direction={direction}, Plane Speed={plane_speed}, Rotation Speed={rotation_speed}")

    def move(self, desired_angle, direction, plane_speed=8192):
        """Move the robot towards a desired angle."""
        angle_error = desired_angle - self.Z_gyro_new
        kP = -50
        adjusted_rotation_speed = kP * angle_error
        if adjusted_rotation_speed > 8192:
            adjusted_rotation_speed = 8192
        if adjusted_rotation_speed < -8192:
            adjusted_rotation_speed = -8192
        self.send_driving_data(direction, plane_speed, adjusted_rotation_speed)
    
    def time_counting_move(self, duration, desired_angle, direction, plane_speed=8192):
        """Move the robot for a specific duration while maintaining a desired angle."""
        start_time = time.time()
        elapsed_time = 0

        while elapsed_time < duration:
            self.move(desired_angle, direction, plane_speed)
            elapsed_time = time.time() - start_time 
    
    def rotate_to(self, desired_angle, direction=0, plane_speed=0):
        """Rotate the robot to a desired angle."""
        while abs(desired_angle - self.Z_gyro) > 3:
            angle_error = desired_angle - self.Z_gyro
            if angle_error > 180:
                angle_error -= 360
            elif angle_error < -180:
                angle_error += 360
            
            if angle_error > 0:
                adjusted_rotation_speed = 8192  # Maximum speed to the right
            else:
                adjusted_rotation_speed = -8192  # Maximum speed to the left

            self.send_driving_data(direction, plane_speed, adjusted_rotation_speed)
    
    def stop(self, duration):
        """Stop the robot for a specified duration."""
        start_time = time.time()
        elapsed_time = 0

        while elapsed_time < duration:
            self.send_driving_data(0, 0, 0)
            elapsed_time = time.time() - start_time  
    
    def red_to_area2(self):
        """Navigate the red path to area 2."""
        while self.X_gyro > -3.0:
            self.get_logger().info("To ramp")
            self.move(0, 0)
        while self.X_gyro < -2.0:
            self.get_logger().info("Incline")
            self.move(0, 0, 5465)
        self.get_logger().info("Inclined")
        
    def red_area2_to_area3(self):
        """Navigate from red area 2 to area 3."""
        while self.Y_distance > 300:
            self.get_logger().info("Area 2-1")
            self.move(0, 0, 4096)
        self.stop(1)
        while self.X_distance < 3985:
            self.get_logger().info("Area 2-2")
            self.move(0, 90, 4096)
        self.stop(1)
        while self.X_gyro > -5.0:
            self.get_logger().info("To ramp")
            self.move(0, 0)
        while self.X_gyro < -1.0:
            self.get_logger().info("Incline")
            self.move(0, 0)
        self.get_logger().info("Inclined")
        while self.Y_distance > 1710:
            self.get_logger().info("Area 3-1")
            self.move(0, 0)
        self.stop(1)
        self.stepper_motor_start()
        self.rotate_to(270)
        self.get_logger().info("Rotate to storage")
        self.stop(1)
        for i in range(6):
            self.time_counting_move(3, -90, 180)
            locked_ball = False
            while True:
                while True:
                    largest_target = self.find_largest_target(self.ai_ball_data, "red")
                    if len(largest_target) > 0:
                        break
                print(1)
                if largest_target["x"] < 310:
                    self.send_driving_data(0, 0, -4096)
                    print(2)
                elif largest_target["x"] > 330:
                    self.send_driving_data(0, 0, 4096)
                    print(3)
                else:
                    print(4)
                    if locked_ball == False:
                        angle_to_target = self.Z_gyro
                        locked_ball = True
                    if largest_target["area"] < 145000:  # Verify this threshold
                        self.move(angle_to_target, 180, 4096)
                    else:
                        self.gripper_control(True)
                        self.stop(1)
                        break
            self.stepper_motor_up()
            self.rotate_to(270)
            self.stop(1)
            while self.X_gyro > -5.0:
                self.get_logger().info("To ramp")
                self.move(-90, 0)
            while self.X_gyro < -1.0:
                self.get_logger().info("Incline")
                self.move(-90, 0)
            self.get_logger().info("Inclined")
            self.stop(1)
            selected_silo_x = self.random_silo()
            while self.X_distance < selected_silo_x - 10 or self.X_distance > selected_silo_x + 10:
                self.get_logger().info("Silo X")
                if self.X_distance < selected_silo_x - 10:
                    self.move(-90, 270, 4096)
                elif self.X_distance > selected_silo_x + 10:
                    self.move(-90, 90, 4096)
            self.stop(1)
            while self.Y_distance > 150 or self.Y_distance < 130:
                self.get_logger().info("Silo Y")
                if self.Y_distance > 150:
                    self.move(-90, 0, 4096)
                elif self.Y_distance < 130:
                    self.move(-90, 180, 4096)
            self.stop(1)
            self.gripper_control(False)
            self.stepper_motor_down()
            self.time_counting_move(3, -90, 180)
        
    def blue_to_area2(self):
        """Navigate the red path to area 2."""
        while self.X_gyro > -3.0:
            self.get_logger().info("To ramp")
            self.move(0, 0)
        while self.X_gyro < -2.0:
            self.get_logger().info("Incline")
            self.move(0, 0, 5465)
        self.get_logger().info("Inclined")
        
        
    def blue_area2_to_area3(self):
        """Navigate from blue area 2 to area 3."""
        while self.Y_distance > 300:
            self.get_logger().info("Area 2-1")
            self.move(0, 0, 4096)
        self.stop(1)
        while self.X_distance < 3985:
            self.get_logger().info("Area 2-2")
            self.move(0, 270, 4096)
        self.stop(1)
        while self.X_gyro > -5.0:
            self.get_logger().info("To ramp")
            self.move(0, 0)
        while self.X_gyro < -1.0:
            self.get_logger().info("Incline")
            self.move(0, 0)
        self.get_logger().info("Inclined")
        while self.Y_distance > 1810:
            self.get_logger().info("Area 3-1")
            self.move(0, 0, 4096)
        self.stop(1)
        self.stepper_motor_start()
        self.rotate_to(90)
        self.get_logger().info("Rotate to storage")
        self.stop(1)
        for i in range(6):
            self.time_counting_move(3, 90, 180)
            locked_ball = False
            while True:
                while True:
                    largest_target = self.find_largest_target(self.ai_ball_data, "blue")
                    if len(largest_target) > 0:
                        break
                print(1)
                if largest_target["x"] < 310:
                    self.send_driving_data(0, 0, -4096)
                    print(2)
                elif largest_target["x"] > 330:
                    self.send_driving_data(0, 0, 4096)
                    print(3)
                else:
                    print(4)
                    if locked_ball == False:
                        angle_to_target = self.Z_gyro
                        locked_ball = True
                    if largest_target["area"] < 145000:  # Verify this threshold
                        self.move(angle_to_target, 180, 4096)
                    else:
                        self.gripper_control(True)
                        self.stop(1)
                        break
            self.stepper_motor_up()
            self.rotate_to(90)
            self.stop(1)
            while self.X_gyro > -5.0:
                self.get_logger().info("To ramp")
                self.move(90, 0)
            while self.X_gyro < -1.0:
                self.get_logger().info("Incline")
                self.move(90, 0)
            self.get_logger().info("Inclined")
            self.stop(1)
            selected_silo_x = self.random_silo()
            while self.X_distance < selected_silo_x - 10 or self.X_distance > selected_silo_x + 10:
                self.get_logger().info("Silo X")
                if self.X_distance < selected_silo_x - 10:
                    self.move(90, 90, 2048)
                elif self.X_distance > selected_silo_x + 10:
                    self.move(90, 270, 2048)
            self.stop(1)
            while self.Y_distance > 150 or self.Y_distance < 130:
                self.get_logger().info("Silo Y")
                if self.Y_distance > 150:
                    self.move(90, 0, 2048)
                elif self.Y_distance < 130:
                    self.move(90, 180, 2048)
            self.stop(1)
            self.gripper_control(False)
            self.stepper_motor_down()
            self.time_counting_move(3, 90, 180)

    def find_largest_target(self, target, target_color):
        """Find the largest target of a specific color."""
        largest_target = None
        max_area = -1
        for item in target:
            color, x, y, area = item
            if color == target_color and area > max_area:
                largest_target = {
                    'color': color,
                    'x': x,
                    'y': y,
                    'area': area
                }
                max_area = area
        return largest_target

    def gripper_control(self, state):
        """Control the gripper relay."""
        GPIO.output(self.relay_pin, GPIO.HIGH if state else GPIO.LOW)

    def stepper_motor_rotate_to_angle(self, angle):
        """Rotate the stepper motor to a specific angle."""
        steps_per_revolution = 800  # Adjust for your motor
        steps = int(abs(angle) / 18.0 * steps_per_revolution)
        GPIO.output(self.dir_pin, GPIO.HIGH if angle > 0 else GPIO.LOW)

        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.speed_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(self.speed_delay)

    def stepper_motor_init(self):
        """Initialize the stepper motor to the starting position."""
        self.stepper_motor_rotate_to_angle(65)

    def stepper_motor_start(self):
        """Start the stepper motor."""
        self.stepper_motor_rotate_to_angle(-65)

    def stepper_motor_up(self):
        """Move the stepper motor up."""
        self.stepper_motor_rotate_to_angle(145)

    def stepper_motor_down(self):
        """Move the stepper motor down."""
        self.stepper_motor_rotate_to_angle(-145)

    def random_silo(self):
        """Choose a random silo that has space."""
        while True:
            chosen_silo = random.randint(0, 2)
            if self.silo_status[chosen_silo] < 3:
                self.silo_status[chosen_silo] += 1
                return self.silo_distances[chosen_silo]

def main(args=None):
    rclpy.init(args=args)
    r2_main = R2Main()
    try:
        r2_main.subscription_thread.join()
        r2_main.start_thread.join()
        r2_main.serial_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        r2_main.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
