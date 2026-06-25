#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import serial
import time


class RoboteqDriverNode(Node):
    def __init__(self):
        super().__init__('roboteq_driver_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyMotor')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_rpm', 100)  # scale factor

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_rpm = self.get_parameter('max_rpm').value

        # Serial connection
        self.ser = None
        self.connect_serial()

        # Subscribers
        self.left_sub = self.create_subscription(
            Float32,
            '/left_wheel_speed',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Float32,
            '/right_wheel_speed',
            self.right_callback,
            10
        )

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.get_logger().info("Roboteq Driver Node Started")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")

    def left_callback(self, msg):
        self.left_speed = msg.data
        self.send_command()

    def right_callback(self, msg):
        self.right_speed = msg.data
        self.send_command()

    def send_command(self):
        if self.ser is None:
            return

        try:
            # Convert m/s → Roboteq scale
            left_cmd = int(self.left_speed * self.max_rpm)
            right_cmd = int(self.right_speed * self.max_rpm)

            # Clamp values
            left_cmd = max(min(left_cmd, self.max_rpm), -self.max_rpm)
            right_cmd = max(min(right_cmd, self.max_rpm), -self.max_rpm)

            # Roboteq command format
            cmd_left = f"!G 1 {left_cmd}\r"
            cmd_right = f"!G 2 {right_cmd}\r"

            self.ser.write(cmd_left.encode())
            self.ser.write(cmd_right.encode())

            self.get_logger().info(
                f"Sent -> L:{left_cmd} R:{right_cmd}"
            )

        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self.reconnect()

    def reconnect(self):
        self.get_logger().warn("Reconnecting serial...")
        try:
            if self.ser:
                self.ser.close()
            time.sleep(1)
            self.connect_serial()
        except Exception as e:
            self.get_logger().error(f"Reconnect failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()