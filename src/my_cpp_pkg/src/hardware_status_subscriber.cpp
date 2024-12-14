#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

class LedPanelNode : public rclcpp::Node {
   public:
    LedPanelNode() : Node("a_new_name") {
        subscriber_ =
            this->create_subscription<my_robot_interfaces::msg::HardwareStatus>(
                "hardware_status", 10,
                std::bind(&LedPanelNode::subscriptionCallback, this,
                          std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    void subscriptionCallback(
        const my_robot_interfaces::msg::HardwareStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "msg %ld, %s, %s", msg->temperature,
                    msg->are_motors_ready ? "true" : "false",
                    msg->debug_message.c_str());
    }

    rclcpp::Subscription<my_robot_interfaces::msg::HardwareStatus>::SharedPtr
        subscriber_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}