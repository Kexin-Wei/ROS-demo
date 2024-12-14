#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name = "Py Robot"
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Rbot New Station Node has been started.")

    def publish_news(self):
        msg = String()
        msg.data = f"Hi, this is Robot News Station from {self.robot_name}"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
