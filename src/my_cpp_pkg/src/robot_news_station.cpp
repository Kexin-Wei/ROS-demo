#include <chrono>
#include <memory>
#include <string>

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotNewsStation : public rclcpp::Node {
   public:
    RobotNewsStation() : Node("robot_news_station") {
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");

        publisher_ = this->create_publisher<example_interfaces::msg::String>(
            "robot_news", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotNewsStation::publishNews, this));

        RCLCPP_INFO(this->get_logger(), "Robot news has been started.");
    }

   private:
    void publishNews() {
        auto message = example_interfaces::msg::String();
        message.data = "Hi, this is Robot News Station from " + robotName;
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
        // message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string robotName = "Cpp Robot";
};

int main(int argv, char** argc) {
    rclcpp::init(argv, argc);
    auto node = std::make_shared<RobotNewsStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
