import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct

PORT1 = '/dev/ttyACM1'
PORT2 = '/dev/ttyACM2'
BAUD_RATE = 230400

class STP32LNode(Node):

    def __init__(self):
        super().__init__('stp32l')
        self.publisher_ = self.create_publisher(String, 'stp32l', 10)
        self.ser1 = serial.Serial(PORT1, BAUD_RATE, timeout=1)
        self.ser2 = serial.Serial(PORT2, BAUD_RATE, timeout=1)
        self.sensor_addresses = 0x00

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def send_get_distance_command(self, ser, device_address):
        start = [0xAA, 0xAA, 0xAA, 0xAA]
        command = 0x02
        offset = 0x0000
        length = 0x0000

        packet = start + [device_address, command, offset & 0xFF, (offset >> 8) & 0xFF, length & 0xFF, (length >> 8) & 0xFF]
        checksum = self.calculate_checksum(packet[4:])
        packet.append(checksum)

        ser.write(bytearray(packet))

    def receive_distance_data(self, ser, device_address):
        header = ser.read(10)
        if len(header) < 10:
            self.get_logger().error(f"Received header too short from device {device_address}.")
            return None

        if header[:4] != b'\xAA\xAA\xAA\xAA':
            self.get_logger().error(f"Invalid start symbol from device {device_address}.")
            return None

        device_address, command, offset, length = struct.unpack('<BBHH', header[4:10])
        expected_length = 10 + length + 1

        remaining_data = ser.read(expected_length - 10)
        if len(remaining_data) != (expected_length - 10):
            self.get_logger().error(f"Received data length mismatch from device {device_address}.")
            return None

        data = header + remaining_data

        checksum = data[10 + length]
        calculated_checksum = self.calculate_checksum(data[4:10 + length])

        if checksum != calculated_checksum:
            self.get_logger().error(f"Checksum error from device {device_address}.")
            return None

        start_idx = 10 + 15
        end_idx = start_idx + 15

        distance, noise, peak, confidence, intg, reftof = struct.unpack('<HHLBHL', data[start_idx:end_idx])
        return distance

    def run(self):
        while rclpy.ok():
            self.send_get_distance_command(self.ser1, self.sensor_addresses)
            result1 = self.receive_distance_data(self.ser1, self.sensor_addresses)

            self.send_get_distance_command(self.ser2, self.sensor_addresses)
            result2 = self.receive_distance_data(self.ser2, self.sensor_addresses)

            distances_dataset = f"DTy;{result2};DTx;{result1}"
            msg = String()
            msg.data = distances_dataset
            self.publisher_.publish(msg)
            self.get_logger().info(distances_dataset)

def main(args=None):
    rclpy.init(args=args)
    node = STP32LNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
