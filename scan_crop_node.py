#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCropNode(Node):
    def __init__(self):
        super().__init__('scan_crop_node')

        self.declare_parameter('input_topic', '/front/scan')
        self.declare_parameter('output_topic', '/front/scan_crop')
        # self.declare_parameter('input_topic', '/rear/scan')
        # self.declare_parameter('output_topic', '/rear/scan_crop')
        self.declare_parameter('lower_angle', -1.57)
        self.declare_parameter('upper_angle', 1.57)
        self.declare_parameter('min_range_cutoff', 0.30)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.lower_angle = self.get_parameter('lower_angle').value
        self.upper_angle = self.get_parameter('upper_angle').value
        self.min_range_cutoff = self.get_parameter('min_range_cutoff').value

        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            10
        )

        self.get_logger().info('Scan Crop Node Started')

    def scan_callback(self, msg):
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = max(msg.range_min, self.min_range_cutoff)
        new_msg.range_max = msg.range_max
        new_msg.intensities = msg.intensities

        new_ranges = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if angle < self.lower_angle or angle > self.upper_angle:
                new_ranges.append(float('inf'))
            elif r < self.min_range_cutoff:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(r)

        new_msg.ranges = new_ranges
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCropNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
