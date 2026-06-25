#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('wheel_base', 0.60)

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('deadband', 0.001)

        # Calibration factors
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 0.30)

        self.wheel_base = self.get_parameter('wheel_base').value

        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadband = self.get_parameter('deadband').value

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        # cmd_vel values
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # Time
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_time = now

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscriber
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update_odometry
        )

        self.get_logger().info(
            'CMD_VEL Open-Loop Odometry Node Started'
        )

    def cmd_vel_callback(self, msg):

        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

        self.last_cmd_time = self.get_clock().now()

        # Immediate stop when teleop sends zero
        if abs(self.cmd_linear) < 1e-6 and abs(self.cmd_angular) < 1e-6:

            self.cmd_linear = 0.0
            self.cmd_angular = 0.0

    def update_odometry(self):

        current_time = self.get_clock().now()

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        if dt <= 0.0 or dt > 1.0:
            self.last_time = current_time
            return

        cmd_age = (
            current_time - self.last_cmd_time
        ).nanoseconds / 1e9

        # If no recent /cmd_vel, stop odometry
        if cmd_age > self.cmd_vel_timeout:

            self.cmd_linear = 0.0
            self.cmd_angular = 0.0

        # Deadband
        if abs(self.cmd_linear) < self.deadband:
            self.cmd_linear = 0.0

        if abs(self.cmd_angular) < self.deadband:
            self.cmd_angular = 0.0

        # Open-loop odometry from /cmd_vel
        v = self.cmd_linear * self.linear_scale

        omega = (
            self.cmd_angular *
            self.angular_scale
        )

        d_center = v * dt
        d_theta = omega * dt

        self.x += (
            d_center *
            math.cos(self.theta + d_theta / 2.0)
        )

        self.y += (
            d_center *
            math.sin(self.theta + d_theta / 2.0)
        )

        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # Quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Odometry message
        odom_msg = Odometry()

        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Velocity
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        # Publish odom
        self.odom_pub.publish(odom_msg)

        # TF transform
        t = TransformStamped()

        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time


def main(args=None):

    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()