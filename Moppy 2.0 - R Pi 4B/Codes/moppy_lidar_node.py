#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import serial
import math
import time

class MoppyLidarNode(Node):
    def __init__(self):
        super().__init__('moppy_lidar_node')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(LaserScan, '/scan', qos)
        
        self.port_name = '/dev/ttyUSB0'
        self.ser = None
        self.connect_serial()

        self.num_bins = 360
        self.scan_buffer = [float('inf')] * self.num_bins
        self.last_angle = 0.0
        
        self.timer = self.create_timer(0.02, self.read_lidar_data)

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port_name, 115200, timeout=0.01)
            self.get_logger().info(f'Successfully connected to LiDAR on {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Waiting for LiDAR on {self.port_name}... Check connection.')

    def read_lidar_data(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
            time.sleep(1.0)
            return

        try:
            # Loop until the buffer is drained of all full packets
            while self.ser.in_waiting >= 58:
                byte = self.ser.read(1)
                if byte != b'\xaa':
                    continue

                header = self.ser.read(5)
                if len(header) < 5 or header[4] != 0xad:
                    continue

                param_len = self.ser.read(2)
                param_head = self.ser.read(5)
                if len(param_len) < 2 or len(param_head) < 5:
                    continue

                start_angle = ((param_head[3] << 8) | param_head[4]) / 100.0

                if start_angle < self.last_angle and (self.last_angle - start_angle) > 180.0:
                    self.publish_full_scan()
                self.last_angle = start_angle

                sample_bytes = self.ser.read(45)
                if len(sample_bytes) < 45:
                    continue

                angle_step = 1.5
                for i in range(15):
                    dist_high = sample_bytes[i * 3 + 1]
                    dist_low = sample_bytes[i * 3 + 2]
                    dist_raw = (dist_high << 8) | dist_low
                    dist_m = (dist_raw * 0.25) / 1000.0

                    current_angle = (start_angle + (i * angle_step)) % 360.0
                    bin_idx = int(current_angle) % self.num_bins

                    if 0.1 <= dist_m <= 12.0:
                        self.scan_buffer[bin_idx] = dist_m

        except OSError:
            self.get_logger().error('Hardware disconnected. Retrying connection...')
            self.ser.close()
        except Exception as e:
            self.get_logger().error(f'Serial parser exception: {e}')

    def publish_full_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / self.num_bins
        msg.time_increment = (1.0 / 5.0) / self.num_bins
        msg.scan_time = 1.0 / 5.0
        msg.range_min = 0.1
        msg.range_max = 12.0
        
        msg.ranges = list(self.scan_buffer)
        self.publisher_.publish(msg)
        
        self.scan_buffer = [float('inf')] * self.num_bins

def main(args=None):
    rclpy.init(args=args)
    node = MoppyLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()