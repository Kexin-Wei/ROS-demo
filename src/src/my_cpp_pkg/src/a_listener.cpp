#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class AListener : public rclcpp::Node {
   public:
    AListener() : Node("a_listener") {
        subscriber_ =
            this->create_subscription<example_interfaces::msg::String>(
                "robot_news", 10,
                std::bind(&AListener::callback, this,
                          std::placeholders::_1));  // callback has 1 parameter
        RCLCPP_INFO(this->get_logger(), "Cpp listener has started to listen.");
    }

   private:
    void callback(const example_interfaces::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr
        subscriber_;
};

int main(int argv, char** argc) {
    rclcpp::init(argv, argc);
    auto node = std::make_shared<AListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}