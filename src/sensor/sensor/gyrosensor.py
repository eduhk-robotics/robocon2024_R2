import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import threading

class GyroSensorPublisher(Node):
    def __init__(self):
        super().__init__('gyrosensor')
        self.publisher_ = self.create_publisher(String, '/gyrosensor', 10)
        self.serial_port = '/dev/ttyUSB0'  # Update this to your actual serial port
        self.baud_rate = 115200
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        
        self.data_thread = threading.Thread(target=self.read_serial_data)
        self.data_thread.daemon = True
        self.data_thread.start()

    def calculate_angle(self, high_byte, low_byte):
        value = (high_byte << 8) | low_byte
        if value >= 0x8000:
            value = value - 0x10000
        return value / 32768.0 * 180.0

    def read_serial_data(self):
        while True:
            if self.serial_connection.in_waiting >= 11:
                if self.serial_connection.read() == b'\x55':
                    packet_type = self.serial_connection.read()
                    if packet_type == b'\x53':  # Angle data packet
                        data = self.serial_connection.read(9)
                        if len(data) == 9:
                            (RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH, SUM) = struct.unpack('<BBBBBBBBB', data)
                            
                            # Verify checksum
                            calculated_sum = (0x55 + 0x53 + RollL + RollH + PitchL + PitchH + YawL + YawH + VL + VH) & 0xFF
                            if calculated_sum == SUM:
                                roll_angle = self.calculate_angle(RollH, RollL)
                                pitch_angle = self.calculate_angle(PitchH, PitchL)
                                yaw_angle = self.calculate_angle(YawH, YawL)

                                # Adjust yaw angle to range 0-360
                                if yaw_angle < 0:
                                    yaw_angle += 360.0

                                # Calculate new yaw angle format (0, 180, 0, -180)
                                if yaw_angle > 180.0:
                                    yaw_angle_new_format = yaw_angle - 360.0
                                else:
                                    yaw_angle_new_format = yaw_angle

                                msg = String()
                                msg.data = f"GS;{roll_angle:.2f};{pitch_angle:.2f};{yaw_angle:.2f};{yaw_angle_new_format:.2f}"
                                self.publisher_.publish(msg)
                                self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GyroSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
