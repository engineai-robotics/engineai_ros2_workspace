#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from interface_protocol.msg import BodyVelCmd
import numpy as np
import math
import time


class BodyVelocityPublisher(Node):
    def __init__(self):
        super().__init__('body_velocity_publisher')

        # Create publisher for body velocity commands
        self.publisher_ = self.create_publisher(
            BodyVelCmd,
            '/motion/body_vel_cmd',
            10  # QoS profile depth
        )

        # Set timer to publish at 100Hz (10ms)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize variables
        self.linear_vel = [0.0, 0.0]  # [x, y] velocities in m/s
        self.yaw_vel = 0.0  # yaw velocity in rad/s

        self.get_logger().info('Body Velocity Publisher started')

    def timer_callback(self):
        # Create message
        msg = BodyVelCmd()

        # Fill header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'

        # Set linear velocity - you can modify these values for testing
        # For example, set a forward velocity of 0.1 m/s
        self.linear_vel[0] = 0.1  # Forward velocity (x-axis)
        self.linear_vel[1] = 0.0  # Lateral velocity (y-axis)

        # Set yaw velocity - rotate at 0.1 rad/s (about 5.7 degrees/s)
        self.yaw_vel = 0.1

        # Fill message fields
        msg.linear_velocity = self.linear_vel
        msg.yaw_velocity = self.yaw_vel

        # Publish message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    body_velocity_publisher = BodyVelocityPublisher()

    try:
        rclpy.spin(body_velocity_publisher)
    except KeyboardInterrupt:
        body_velocity_publisher.get_logger().info('Node stopped by keyboard interrupt')
    finally:
        # Destroy the node explicitly
        body_velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
