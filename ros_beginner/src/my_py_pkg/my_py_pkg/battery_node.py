#! /usr/bin/python3
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStatus


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.max_battery_capacity = 100
        self.min_battery_capacity = 0
        self.battery_capacity = 0
        self.dbattery_capacity = -10
        self.n_len_ = 3
        self.timer_ = self.create_timer(0.5, self.callbackBatteryChange)
        self.timer2_ = self.create_timer(2, self.callbackSetLed)
        self.subscriber_ = self.create_subscription(
            LedStatus,
            "led_panel_state",
            self.callback_update_n_led,
            10,
        )
        self.get_logger().info(f"{self.get_name()} is established")

    def callback_update_n_led(self, msg: LedStatus):
        self.n_len_ = len(msg.led_states)

    def correct_index(self, index):
        return np.clip(index, 0, self.n_len_)

    def calc_current_index(self):
        current_index = int(
            (self.battery_capacity - self.min_battery_capacity)
            / (self.max_battery_capacity - self.min_battery_capacity)
            * self.n_len_
        )
        return self.correct_index(current_index)

    def correct_index_with_on_or_off(self):
        control_index = self.calc_current_index()
        set_state = True
        if self.dbattery_capacity < 0:
            control_index = control_index + 1
            set_state = False
        return self.correct_index(control_index), set_state

    def callbackBatteryChange(self):
        if (
            self.battery_capacity == self.max_battery_capacity
            or self.battery_capacity == self.min_battery_capacity
        ):
            self.dbattery_capacity = self.dbattery_capacity * (-1)
        self.battery_capacity = self.battery_capacity + self.dbattery_capacity

    def callbackSetLed(self):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1):
            self.get_logger().info("Waiting server...")

        request = SetLed.Request()
        index, state = self.correct_index_with_on_or_off()
        request.led_index = int(index)
        request.set_state = state
        self.get_logger().info(
            f"battery: {self.battery_capacity}, "
            f"idx: {request.led_index}, set:{request.set_state}"
        )
        future = client.call_async(request=request)
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
