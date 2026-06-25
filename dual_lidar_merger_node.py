#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DualLidarMerger(Node):
    def __init__(self):
        super().__init__('dual_lidar_merger_node')

        self.front_scan = None
        self.rear_scan = None

        self.create_subscription(LaserScan, '/front/scan_crop', self.front_callback, 10)
        self.create_subscription(LaserScan, '/rear/scan_crop', self.rear_callback, 10)

        self.pub = self.create_publisher(LaserScan, '/merge/scan', 10)

        self.timer = self.create_timer(0.05, self.publish_merged_scan)

        self.get_logger().info(
            'Dual LiDAR merger started: /front/scan_crop + /rear/scan_crop -> /merge/scan'
        )

    def front_callback(self, msg):
        self.front_scan = msg

    def rear_callback(self, msg):
        self.rear_scan = msg

    def publish_merged_scan(self):
        if self.front_scan is None or self.rear_scan is None:
            return

        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'base_link'

        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = self.front_scan.angle_increment
        merged.time_increment = self.front_scan.time_increment
        merged.scan_time = self.front_scan.scan_time
        merged.range_min = 0.30
        merged.range_max = 6.0

        total_points = int((merged.angle_max - merged.angle_min) / merged.angle_increment) + 1
        merged.ranges = [float('inf')] * total_points
        merged.intensities = [0.0] * total_points

        # Your corrected diagonal TF yaw:
        # front lidar ≈ -45 degree
        # rear lidar  ≈ +135 degree
        self.insert_scan(merged, self.front_scan, yaw_offset=-math.pi / 4.0)
        self.insert_scan(merged, self.rear_scan, yaw_offset=3.0 * math.pi / 4.0)

        self.pub.publish(merged)

    def insert_scan(self, merged, scan, yaw_offset):
        for i, r in enumerate(scan.ranges):

            if math.isnan(r) or math.isinf(r):
                continue

            if r < merged.range_min or r > merged.range_max:
                continue

            local_angle = scan.angle_min + i * scan.angle_increment

            angle = local_angle + yaw_offset

            while angle > math.pi:
                angle -= 2.0 * math.pi

            while angle < -math.pi:
                angle += 2.0 * math.pi

            index = int((angle - merged.angle_min) / merged.angle_increment)

            if 0 <= index < len(merged.ranges):
                if r < merged.ranges[index]:
                    merged.ranges[index] = r

                    if i < len(scan.intensities):
                        merged.intensities[index] = scan.intensities[i]


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()