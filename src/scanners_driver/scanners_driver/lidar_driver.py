import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import serial
import time
import math

from sensor_msgs.msg import LaserScan

class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = None
        self.buffer = bytearray()

        self.try_connect()

        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        self.timer = self.create_timer(0.01, self.read_serial)
        self.last_data_time = self.get_clock().now()

        self.get_logger().info(f"Listening on {self.port} at {self.baudrate} baud")

    def try_connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)

            self.ser.flushInput()
            self.ser.flushOutput()
            time.sleep(1.0)
            self.get_logger().info(f"Successfully connected to {self.port}", throttle_duration_sec=5.0)
        except serial.SerialException as e:
            self.get_logger().info(f"Failed to open serial port: {e}", throttle_duration_sec=5.0)
            self.ser = None
        except Exception as e:
            self.get_logger().info(f"Unexpected exception while opening port: {e}", throttle_duration_sec=5.0)
            self.ser = None

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().info("Serial port is not open. Trying to connect...", throttle_duration_sec=5.0)
            self.try_connect()
            if self.ser is None or not self.ser.is_open:
                return

        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.last_data_time = self.get_clock().now()            
            self.buffer.extend(data)

            while b"LIDAR" in self.buffer and b"END\n\r" in self.buffer:
                start = self.buffer.find(b"LIDAR")
                end = self.buffer.find(b"END\n\r", start)
                if end == -1:
                    break

                frame = self.buffer[start:end+len(b"END\n\r")]
                self.buffer = self.buffer[end+len(b"END\n\r"):]

                self.parse_frame(frame)
        else:
            if (self.get_clock().now() - self.last_data_time) > Duration(seconds=1.0):
                self.get_logger().info("No data received for 1 second, reseting connection")

                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                    except:
                        pass 
                
                self.try_connect()
                self.last_data_time = self.get_clock().now()

    def parse_frame(self, frame: bytes):
        if len(frame) < 8:
            return

        size = frame[5] | (frame[6] << 8)
        payload = frame[7:-5]

        if len(payload) != size:
            self.get_logger().warn(f"Invalid frame size: expected {size}, got {len(payload)}")
            return

        ranges = []
        intensities = []

        for i in range(0, len(payload), 4):
            if i + 3 < len(payload):
                dist = payload[i] | (payload[i+1] << 8)
                strength = payload[i+2] | (payload[i+3] << 8)

                ranges.append(dist / 100.0)
                intensities.append(float(strength))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(ranges) if ranges else 0.0
        msg.range_min = 0.03
        msg.range_max = 8.0
        msg.ranges = ranges
        msg.intensities = intensities

        self.pub.publish(msg)
        self.get_logger().info(f"Published scan with {len(ranges)} points")


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

