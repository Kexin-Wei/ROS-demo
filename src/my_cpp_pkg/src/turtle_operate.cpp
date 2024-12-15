#include "rclcpp/rclcpp.hpp"

class LedPanelNode : public rclcpp::Node {
   public:
    LedPanelNode() : Node("a_new_name") {
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}