#!  /usr/bin/python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.get_logger().info("AddTwoIntsClient is established.")
        self.call_add_two_ints_server(10, 2)
        self.call_add_two_ints_server(4, 8)

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = client.call_async(request=request)
        future.add_done_callback(partial(self.callback_future, request=request))

    def callback_future(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(f"{request.a}+{request.b}={response.sum}")
        except Exception as e:
            self.get_logger().error(f"Error occured {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
