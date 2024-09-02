import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoMegaM3508CommNode(Node):
    def __init__(self):
        super().__init__('arduino_mega_m3508_comm_node')
        self.publisher_ = self.create_publisher(String, 'motor_data', 10)
        self.subscription = self.create_subscription(
            String,
            'motor_control',
            self.motor_control_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust the port to match your setup
        self.timer = self.create_timer(0.01, self.timer_callback)  # Add delay of 1 second for reading Arduino data
        self.control_timer = self.create_timer(0.01, self.control_timer_callback)  # Add delay of 2 seconds for publishing control messages

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            arduino_data = self.ser.readline().decode('utf-8').strip()
            msg = String()
            msg.data = arduino_data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing motor data: "%s"' % msg.data)

    def motor_control_callback(self, msg):
    	try:
        	# Log the received control command
        	command = msg.data
        	self.get_logger().info('Received control command: "%s"' % command)
        
        	# Send the command to the Arduino
        	self.ser.write((command + '\n').encode('utf-8'))
        	self.get_logger().info('Sent to Arduino: "%s"' % command)
        
        	# Ensure the command is actually sent
        	time.sleep(0.1)  # Give some time for the command to be processed
        
        	# Check for response from Arduino
        	if self.ser.in_waiting > 0:
            		response = self.ser.readline().decode('utf-8').strip()
            		self.get_logger().info('Arduino response: "%s"' % response)
        	else:
            		self.get_logger().info('No response from Arduino')
    
    	except Exception as e:
        	self.get_logger().error('Error in motor_control_callback: %s' % str(e))


    def control_timer_callback(self):
        motor_id = 1  # Example motor ID
        power = 1000  # Example power value
        command = f"{motor_id},{power}"
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing control command: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMegaM3508CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
