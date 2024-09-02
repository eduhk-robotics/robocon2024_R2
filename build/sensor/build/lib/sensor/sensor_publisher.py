import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorDataAggregator(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(String, 'sensor', 10)
        
        # Subscribers
        self.stp32l_subscriber = self.create_subscription(
            String,
            'stp32l',
            self.stp32l_callback,
            10)
        self.gyrosensor_subscriber = self.create_subscription(
            String,
            'gyrosensor',
            self.gyrosensor_callback,
            10)
        
        # Sensor data storage
        self.stp32l_data = None
        self.gyrosensor_data = None

    def stp32l_callback(self, msg):
        self.stp32l_data = msg.data
        self.publish_combined_data()

    def gyrosensor_callback(self, msg):
        self.gyrosensor_data = msg.data
        self.publish_combined_data()

    def publish_combined_data(self):
        if self.stp32l_data is not None and self.gyrosensor_data is not None:
            combined_data = f"{self.stp32l_data};{self.gyrosensor_data}"
            self.get_logger().info(combined_data)
            msg = String()
            msg.data = combined_data
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_data_aggregator = SensorDataAggregator()
    rclpy.spin(sensor_data_aggregator)
    sensor_data_aggregator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
