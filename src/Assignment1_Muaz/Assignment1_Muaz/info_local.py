#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from assignment1_interfaces.msg import RobotInfo
import random


class InfoLocalNode(Node):
    def __init__(self):
        super().__init__('info_local')

        # Create publisher for robot information
        self.publisher_ = self.create_publisher(RobotInfo, 'robot_info_local', 10)

        # Create timer to publish at 1 Hz (every 1 second)
        self.timer = self.create_timer(1.0, self.publish_robot_info)

        self.get_logger().info('Info Local Node has been started')

    def publish_robot_info(self):
        msg = RobotInfo()

        # Robot name
        msg.robot_name = 'robot1'

        # Arbitrary position in local frame (you can modify these values)
        msg.x = 5.0 + random.uniform(-0.5, 0.5)  # Adding some variation
        msg.y = 3.0 + random.uniform(-0.5, 0.5)
        msg.theta = 30.0  # degrees

        # Current temperature (simulated)
        msg.temperature = 25.0 + random.uniform(-2.0, 2.0)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: robot={msg.robot_name}, pos=({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f}°), temp={msg.temperature:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = InfoLocalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
