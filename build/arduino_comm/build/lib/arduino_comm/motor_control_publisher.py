import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorControlPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_control', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Add delay of 2 seconds

    def timer_callback(self):
        motor_id = 1  # Example motor ID
        power = 1000  # Example power value
        command = f"{motor_id},{power}"
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
