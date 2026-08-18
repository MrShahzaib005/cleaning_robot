#!/usr/bin/env python3
"""
Ultrasonic LaserScan → PointCloud2 Converter Node
8 ultrasonic LaserScan topics → 1 PointCloud2 for Nav2.

FIX: Gazebo plugins now output LaserScan instead of Range.
LaserScan works perfectly in ROS2 Humble Python — NO CRASH.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math


class UltrasonicToPointCloud(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_pointcloud')

        self.topics = [
            '/ultrasonic/front_right',
            '/ultrasonic/front_left',
            '/ultrasonic/back_right',
            '/ultrasonic/back_left',
            '/ultrasonic/rightside_front',
            '/ultrasonic/rightside_back',
            '/ultrasonic/leftside_front',
            '/ultrasonic/leftside_back',
        ]

        self.positions = {
            '/ultrasonic/front_right':     ( 0.27, -0.17, 0.245),
            '/ultrasonic/front_left':      ( 0.27,  0.17, 0.245),
            '/ultrasonic/back_right':      (-0.27, -0.15, 0.245),
            '/ultrasonic/back_left':       (-0.27,  0.15, 0.245),
            '/ultrasonic/rightside_front': ( 0.20, -0.20, 0.245),
            '/ultrasonic/rightside_back':  (-0.20, -0.20, 0.245),
            '/ultrasonic/leftside_front':  ( 0.20,  0.20, 0.245),
            '/ultrasonic/leftside_back':   (-0.20,  0.20, 0.245),
        }

        self.directions = {
            '/ultrasonic/front_right':     ( 1.0,  0.0, 0.0),
            '/ultrasonic/front_left':      ( 1.0,  0.0, 0.0),
            '/ultrasonic/back_right':      (-1.0,  0.0, 0.0),
            '/ultrasonic/back_left':       (-1.0,  0.0, 0.0),
            '/ultrasonic/rightside_front': ( 0.0, -1.0, 0.0),
            '/ultrasonic/rightside_back':  ( 0.0, -1.0, 0.0),
            '/ultrasonic/leftside_front':  ( 0.0,  1.0, 0.0),
            '/ultrasonic/leftside_back':   ( 0.0,  1.0, 0.0),
        }

        self.min_ranges = {}

        # Subscribe to LaserScan — NOT Range (Range crashes Python in Humble)
        self.create_subscription(LaserScan, '/ultrasonic/front_right',
            self.cb_front_right, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/front_left',
            self.cb_front_left, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/back_right',
            self.cb_back_right, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/back_left',
            self.cb_back_left, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/rightside_front',
            self.cb_rightside_front, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/rightside_back',
            self.cb_rightside_back, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/leftside_front',
            self.cb_leftside_front, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/ultrasonic/leftside_back',
            self.cb_leftside_back, qos_profile_sensor_data)

        self.pub = self.create_publisher(PointCloud2, '/ultrasonic/pointcloud', 10)
        self.create_timer(0.1, self.publish_pointcloud)

        self.get_logger().info('Ultrasonic to PointCloud2 node started (LaserScan mode)!')
        self.get_logger().info('Subscribed to 8 ultrasonic LaserScan topics')

    def process_scan(self, msg, topic):
        """LaserScan se minimum range nikalo"""
        min_range = float('inf')
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                if r < min_range:
                    min_range = r
        if min_range < float('inf'):
            self.min_ranges[topic] = min_range
        else:
            self.min_ranges.pop(topic, None)

    def cb_front_right(self, msg): self.process_scan(msg, '/ultrasonic/front_right')
    def cb_front_left(self, msg): self.process_scan(msg, '/ultrasonic/front_left')
    def cb_back_right(self, msg): self.process_scan(msg, '/ultrasonic/back_right')
    def cb_back_left(self, msg): self.process_scan(msg, '/ultrasonic/back_left')
    def cb_rightside_front(self, msg): self.process_scan(msg, '/ultrasonic/rightside_front')
    def cb_rightside_back(self, msg): self.process_scan(msg, '/ultrasonic/rightside_back')
    def cb_leftside_front(self, msg): self.process_scan(msg, '/ultrasonic/leftside_front')
    def cb_leftside_back(self, msg): self.process_scan(msg, '/ultrasonic/leftside_back')

    def publish_pointcloud(self):
        points = []

        for topic, distance in self.min_ranges.items():
            if distance < 0.5:
                continue

            sx, sy, sz = self.positions[topic]
            dx, dy, dz = self.directions[topic]

            px = sx + dx * distance
            py = sy + dy * distance
            pz = sz

            points.append((px, py, pz))

        if not points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_footprint'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        data = bytearray()
        for (x, y, z) in points:
            data += struct.pack('fff', x, y, z)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.data = bytes(data)
        cloud.is_dense = True

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
