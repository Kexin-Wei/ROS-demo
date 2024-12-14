#include <chrono>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_opp");

    auto client = node->create_client<example_interfaces::srv::AddTwoInts>(
        "add_two_ints");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for server...");
    }

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 9;
    request->b = 2;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), " %ld + %ld = %ld", request->a,
                    request->b, future.get()->sum);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Error.");
    }

    rclcpp::shutdown();
    return 0;
}