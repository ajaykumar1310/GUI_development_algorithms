#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')

        # Parameters
        self.declare_parameter('wheel_separation', 0.60)   # meters
        self.declare_parameter('max_wheel_speed', 2.0)     # m/s

        self.wheel_separation = self.get_parameter(
            'wheel_separation').get_parameter_value().double_value
        self.max_wheel_speed = self.get_parameter(
            'max_wheel_speed').get_parameter_value().double_value

        # Subscriber to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for left and right wheel speeds
        self.left_wheel_pub = self.create_publisher(
            Float32,
            '/left_wheel_speed',
            10
        )

        self.right_wheel_pub = self.create_publisher(
            Float32,
            '/right_wheel_speed',
            10
        )

        self.get_logger().info('Differential Drive Node Started')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation} m')
        self.get_logger().info(f'Max wheel speed: {self.max_wheel_speed} m/s')

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Differential drive equations
        left_speed = linear_velocity - (angular_velocity * self.wheel_separation / 2.0)
        right_speed = linear_velocity + (angular_velocity * self.wheel_separation / 2.0)

        # Limit speeds to max_wheel_speed
        left_speed = max(min(left_speed, self.max_wheel_speed), -self.max_wheel_speed)
        right_speed = max(min(right_speed, self.max_wheel_speed), -self.max_wheel_speed)

        # Publish wheel speeds
        left_msg = Float32()
        right_msg = Float32()

        left_msg.data = float(left_speed)
        right_msg.data = float(right_speed)

        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

        self.get_logger().info(
            f'/cmd_vel -> linear.x: {linear_velocity:.2f}, angular.z: {angular_velocity:.2f} | '
            f'left: {left_speed:.2f}, right: {right_speed:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
