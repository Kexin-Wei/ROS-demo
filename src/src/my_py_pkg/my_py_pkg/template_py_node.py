#! /usr/bin/python3
import rclpy
from rclpy.node import Node


class NewNode(Node):
    def __init__(self):
        super().__init__("a_new_name")
        self.get_logger().info(f"{self.get_name()} is established")


def main(args=None):
    rclpy.init(args=args)
    node = NewNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
