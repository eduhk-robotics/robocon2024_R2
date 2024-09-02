import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorDataSubscriber(Node):
    def __init__(self):
        super().__init__('motor_data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'motor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        rclpy.sleep(0.1)  # Optional: Add a small delay of 0.1 seconds

def main(args=None):
    rclpy.init(args=args)
    node = MotorDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
