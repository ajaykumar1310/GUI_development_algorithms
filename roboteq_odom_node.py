#!/usr/bin/env python3

import math
import re
import time
import serial
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class RoboteqOdomNode(Node):
    def __init__(self):
        super().__init__('roboteq_odom_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('port', '/dev/ttyMotor')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('wheel_radius', 0.14)       # meter
        self.declare_parameter('wheel_base', 0.60)         # meter
        self.declare_parameter('counts_per_rev', 4096.0)   # from Roborun+

        self.declare_parameter('publish_rate', 3.0)       # Hz
        self.declare_parameter('cmd_timeout', 0.5)         # sec
        self.declare_parameter('max_cmd', 80)              # safe initial value
        self.declare_parameter('velocity_to_cmd_scale', 100.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value

        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_cmd = int(self.get_parameter('max_cmd').value)
        self.velocity_to_cmd_scale = self.get_parameter('velocity_to_cmd_scale').value

        # -----------------------------
        # Sign correction
        # Change these later if direction is wrong
        # -----------------------------
        self.left_motor_sign = 1
        self.right_motor_sign = 1

        self.left_encoder_sign = 1.0
        self.right_encoder_sign = 1.0

        # -----------------------------
        # Robot pose
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_count = None
        self.prev_right_count = None

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        self.left_cmd = 0
        self.right_cmd = 0

        self.reconnecting = False
        self.last_reconnect_attempt = 0.0
        self.reconnect_interval = 5.0

        # -----------------------------
        # Serial connection
        # -----------------------------
        self.ser = None
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.3,
                write_timeout=0.3
            )
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.get_logger().info(f'Connected to RoboteQ on {self.port}')

        except Exception as e:
            self.get_logger().error(f'Could not connect to RoboteQ: {e}')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info('RoboteQ Base Node Started')
        self.get_logger().info('This node sends motor commands and reads encoders using one serial port.')

    # -------------------------------------------------
    # Receive /cmd_vel
    # -------------------------------------------------
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive inverse kinematics
        left_velocity = linear - (angular * self.wheel_base / 2.0)
        right_velocity = linear + (angular * self.wheel_base / 2.0)

        # Convert desired wheel velocity to open-loop RoboteQ command
        left_cmd = int(left_velocity * self.velocity_to_cmd_scale)
        right_cmd = int(right_velocity * self.velocity_to_cmd_scale)

        # Limit command for safety
        left_cmd = max(min(left_cmd, self.max_cmd), -self.max_cmd)
        right_cmd = max(min(right_cmd, self.max_cmd), -self.max_cmd)

        self.left_cmd = self.left_motor_sign * left_cmd
        self.right_cmd = self.right_motor_sign * right_cmd

        self.last_cmd_time = self.get_clock().now()
    # -------------------------------------------------
    # Additional Functions
    # -------------------------------------------------
    def reconnect_serial(self):
        now = time.time()

        if self.reconnecting:
            return False

        if now - self.last_reconnect_attempt < self.reconnect_interval:
            return False

        self.reconnecting = True
        self.last_reconnect_attempt = now

        self.get_logger().warn('Trying to reconnect RoboteQ serial...')

        try:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            # Wait for /dev/ttyMotor to reappear
            port_ready = False
            for i in range(10):
                if os.path.exists(self.port):
                    port_ready = True
                    break

                self.get_logger().warn(
                    f'Waiting for {self.port} to appear... attempt {i+1}/10'
                )
                time.sleep(0.5)

            if not port_ready:
                self.get_logger().error(f'{self.port} not found after waiting.')
                self.reconnecting = False
                return False

            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.5,
                write_timeout=0.5
            )

            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.get_logger().info(f'Reconnected to RoboteQ on {self.port}')

            self.reconnecting = False
            return True

        except Exception as e:
            self.get_logger().error(f'Reconnect failed: {e}')
            self.ser = None
            self.reconnecting = False
            return False


    # -------------------------------------------------
    # Send motor commands to RoboteQ
    # -------------------------------------------------
    def send_motor_commands(self, left_cmd, right_cmd):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            self.ser.write(f'!G 1 {left_cmd}\r'.encode())
            time.sleep(0.05)

            self.ser.write(f'!G 2 {right_cmd}\r'.encode())
            time.sleep(0.05)

        except Exception as e:
            self.get_logger().warn(f'Motor command failed: {e}')
            self.reconnect_serial()

    # -------------------------------------------------
    # Read encoder count from RoboteQ
    # -------------------------------------------------
    def read_encoder(self, channel):
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            self.ser.reset_input_buffer()

            self.ser.write(f'?C {channel}\r'.encode())
            time.sleep(0.10)

            raw = self.ser.read_all().decode(errors='ignore')

            # Expected raw example:
            # '?C 1\rC=-21528'
            match = re.search(r'C=(-?\d+)', raw)

            if match:
                return int(match.group(1))

            self.get_logger().warn(
                f'No encoder value found channel {channel}: raw="{raw}"'
            )
            return None

        except Exception as e:
            self.get_logger().warn(f'Encoder read failed channel {channel}: {e}')
            self.reconnect_serial()
            return None

    # -------------------------------------------------
    # Normalize angle
    # -------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # -------------------------------------------------
    # Publish /odom and TF odom -> base_link
    # -------------------------------------------------
    def publish_odometry(self, current_time, v, omega):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

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

        self.tf_broadcaster.sendTransform(t)

    # -------------------------------------------------
    # Main update loop
    # -------------------------------------------------
    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Safety: stop if no /cmd_vel received recently
        cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if cmd_age > self.cmd_timeout:
            cmd_left = 0
            cmd_right = 0
        else:
            cmd_left = self.left_cmd
            cmd_right = self.right_cmd

        # 1. Send motor command
        self.send_motor_commands(cmd_left, cmd_right)

        # 2. Read encoder counts
        left_count = self.read_encoder(1)
        right_count = self.read_encoder(2)

        if left_count is None or right_count is None:
            self.last_time = current_time
            return

        # First reading initializes encoder count
        if self.prev_left_count is None or self.prev_right_count is None:
            self.prev_left_count = left_count
            self.prev_right_count = right_count
            self.last_time = current_time
            self.publish_odometry(current_time, 0.0, 0.0)
            return

        delta_left_count = left_count - self.prev_left_count
        delta_right_count = right_count - self.prev_right_count

        self.prev_left_count = left_count
        self.prev_right_count = right_count

        delta_left_count *= self.left_encoder_sign
        delta_right_count *= self.right_encoder_sign

        # Convert encoder counts to wheel distance
        left_distance = (
            2.0 * math.pi * self.wheel_radius *
            delta_left_count / self.counts_per_rev
        )

        right_distance = (
            2.0 * math.pi * self.wheel_radius *
            delta_right_count / self.counts_per_rev
        )

        # Differential drive odometry
        d_center = (left_distance + right_distance) / 2.0
        d_theta = (right_distance - left_distance) / self.wheel_base

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta = self.normalize_angle(self.theta)

        v = d_center / dt
        omega = d_theta / dt

        # 3. Publish odometry and TF
        self.publish_odometry(current_time, v, omega)

        self.last_time = current_time

    # -------------------------------------------------
    # Safe shutdown
    # -------------------------------------------------
    def stop_motors_and_close(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.send_motor_commands(0, 0)
                time.sleep(0.1)
                self.ser.close()
                self.get_logger().info('Motors stopped and serial port closed.')
            except Exception as e:
                self.get_logger().warn(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
