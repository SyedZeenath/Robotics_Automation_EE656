#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__('rx200_moveit_control')

        self.get_logger().info('Node initialized successfully!')


def main():
    rclpy.init()
    node = MoveItEEClient()
    # node.send_pose(0.25, 0.0, 0.15)  # single EE pose
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
