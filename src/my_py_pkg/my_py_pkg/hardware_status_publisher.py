#! /usr/bin/python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hardware_status)
        self.get_logger().info(f"{self.get_name()} is established")

    def publish_hardware_status(self):
        msg = HardwareStatus()
        msg.temperature = 23
        msg.are_motors_ready = True
        msg.debug_message = "Test"
        self.publisher_.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisher()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
