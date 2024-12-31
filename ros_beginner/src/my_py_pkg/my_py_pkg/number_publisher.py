#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class NumberPublisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number", 1)
        self.number_ = self.get_parameter("number").get_parameter_value().integer_value
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("NumberPublisher has started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
