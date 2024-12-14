#! /usr/bin/python3
import rclpy
import threading
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.battery_capacity = 80
        self.dbattery_capacity = 10
        self.timer_ = self.create_timer(0.1, self.callbackBatteryChange)
        self.timer2_ = self.create_timer(0.5, self.callbackSetLed)
        self.get_logger().info(f"{self.get_name()} is established")

    def callbackBatteryChange(self):
        if self.battery_capacity == 80 or self.battery_capacity == 0:
            self.dbattery_capacity = self.dbattery_capacity * (-1)
        self.battery_capacity = self.battery_capacity + self.dbattery_capacity

    def callbackSetLed(self):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1):
            self.get_logger().info("Waiting server...")

        change_index = int(self.battery_capacity / 30)
        request = SetLed.Request()
        request.led_index = change_index
        request.set_state = True if self.dbattery_capacity > 0 else False
        future = client.call_async(request=request)
        self.get_logger().info(
            f"battery: {self.battery_capacity}, "
            f"idx: {request.led_index}, set:{request.set_state}"
        )
        future.add_done_callback(partial(self.callbackFromServer, request=request))

    def callbackFromServer(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(f"Responed: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
