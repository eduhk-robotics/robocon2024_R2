import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.sensor_subscriber = self.create_subscription(String, 'sensor', self.sensor_callback, 10)
        self.auto_move_subscriber = self.create_subscription(String, 'auto_move', self.auto_move_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)
        
        self.current_facing_direction = None
        self.gyro_z = 0
        
        self.get_logger().info("Movement control node initialized")

    def sensor_callback(self, msg):
        data = msg.data.split(';')
        self.dtx = int(data[1])
        self.dty = int(data[3])
        self.gyro_x = int(data[5])
        self.gyro_y = int(data[6])
        self.gyro_z = int(data[7])
        
        if self.current_facing_direction is None:
            self.current_facing_direction = self.gyro_z

    def auto_move_callback(self, msg):
        command = msg.data.split(';')
        if command[0] == 'move':
            self.move(command[1], int(command[2]))
        elif command[0] == 'move_until':
            self.move_until(command[1], int(command[2]), command[3], int(command[4]))
        elif command[0] == 'rotate':
            self.rotate(int(command[1]))

    def move(self, direction, speed):
        direction_map = {'front': 0, 'left': 90, 'right': -90, 'back': 180}
        target_direction = (self.current_facing_direction + direction_map[direction]) % 360
        rotation_speed = 0
        
        deviation = self.gyro_z - self.current_facing_direction
        if deviation > 180:
            deviation -= 360
        elif deviation < -180:
            deviation += 360

        if abs(deviation) > 5:
            if deviation > 0:
                left_deviation = deviation
                right_deviation = 360 - deviation if deviation > 180 else -deviation
            else:
                left_deviation = 360 + deviation if deviation < -180 else -deviation
                right_deviation = -deviation

            if abs(left_deviation) < abs(right_deviation):
                rotation_speed = min(max(left_deviation * 10, -1500), 1500)
            else:
                rotation_speed = min(max(right_deviation * 10, -1500), 1500)
            
            if abs(deviation) > 15:
                speed = 0

        # Cap the plane movement speed to 4096
        speed = min(speed, 4096)

        msg = Float32MultiArray()
        msg.data = [target_direction, speed, rotation_speed]
        self.publisher_.publish(msg)

    def move_until(self, direction, speed, sensor, distance):
        sensor_value = self.dtx if sensor == 'DTx' else self.dty
        if sensor_value <= distance:
            self.move(direction, speed)
        else:
            self.stop()

    def rotate(self, degree):
        self.current_facing_direction = (self.current_facing_direction + degree) % 360
        rotation_speed = min(max(degree * 10, -1500), 1500)
        
        msg = Float32MultiArray()
        msg.data = [0, 0, rotation_speed]
        self.publisher_.publish(msg)

    def stop(self):
        msg = Float32MultiArray()
        msg.data = [0, 0, 0]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovementControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
