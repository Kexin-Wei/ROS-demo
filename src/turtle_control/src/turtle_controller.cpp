#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/teleport_relative.hpp"

class TurtleControllerNode : public rclcpp::Node {
   public:
    TurtleControllerNode() : Node("turtle_controller") {
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}