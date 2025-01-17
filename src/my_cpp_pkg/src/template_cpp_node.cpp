#include "rclcpp/rclcpp.hpp"

class NewNode : public rclcpp::Node {
   public:
    NewNode() : Node("a_new_name") {
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<NewNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}