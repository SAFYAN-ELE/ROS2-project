#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from assignment1_interfaces.msg import RobotInfo
import math


class InfoGlobalNode(Node):
    def __init__(self):
        super().__init__('info_global')

        # Subscriber to local robot info
        self.subscription = self.create_subscription(
            RobotInfo,
            'robot_info_local',
            self.listener_callback,
            10)

        # Publisher for global robot info
        self.publisher_ = self.create_publisher(RobotInfo, 'robot_info_global', 10)

        # Rotation angle between frames (50 degrees)
        self.rotation_angle = 50.0  # degrees

        self.get_logger().info('Info Global Node has been started')
        self.get_logger().info(f'Frame rotation angle: {self.rotation_angle}°')

    def listener_callback(self, msg_local):
        # Create new message for global frame
        msg_global = RobotInfo()

        # Robot name and temperature remain unchanged
        msg_global.robot_name = msg_local.robot_name
        msg_global.temperature = msg_local.temperature

        # Transform position from local to global reference frame
        # Rotation transformation:
        # x_global = x_local * cos(θ) - y_local * sin(θ)
        # y_global = x_local * sin(θ) + y_local * cos(θ)
        # theta_global = theta_local + rotation_angle

        angle_rad = math.radians(self.rotation_angle)

        msg_global.x = msg_local.x * math.cos(angle_rad) - msg_local.y * math.sin(angle_rad)
        msg_global.y = msg_local.x * math.sin(angle_rad) + msg_local.y * math.cos(angle_rad)
        msg_global.theta = msg_local.theta + self.rotation_angle

        # Normalize theta to [0, 360)
        msg_global.theta = msg_global.theta % 360.0

        # Publish global frame data
        self.publisher_.publish(msg_global)

        self.get_logger().info(
            f'Transformed: Local({msg_local.x:.2f}, {msg_local.y:.2f}, {msg_local.theta:.2f}°) -> '
            f'Global({msg_global.x:.2f}, {msg_global.y:.2f}, {msg_global.theta:.2f}°), '
            f'temp={msg_global.temperature:.2f}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    node = InfoGlobalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
