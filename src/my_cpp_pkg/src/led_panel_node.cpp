#include <chrono>
#include "my_robot_interfaces/msg/led_status.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "rclcpp/rclcpp.hpp"

class LedPanelNode : public rclcpp::Node {
   public:
    LedPanelNode() : Node("led_panel_node") {
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led", std::bind(&LedPanelNode::callbackSetLed, this,
                                 std::placeholders::_1, std::placeholders::_2));
        publisher_ =
            this->create_publisher<my_robot_interfaces::msg::LedStatus>(
                "led_panel_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LedPanelNode::publishLedStatusCallback, this));
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    void callbackSetLed(
        const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response) {
        auto idx = request->led_index;
        if (idx > 2 || idx < 0) {
            response->success = false;
            response->message = "No such a led, out of index.";
            return;
        }
        ledState_[idx] = request->set_state;
        response->success = true;
        response->message = "Set done";
        RCLCPP_INFO(this->get_logger(), "%ld set to %s", idx,
                    request->set_state ? "true" : "fasle");
    }

    void publishLedStatusCallback() {
        auto msg = my_robot_interfaces::msg::LedStatus();
        msg.led1 = ledState_[0];
        msg.led2 = ledState_[1];
        msg.led3 = ledState_[2];
        publisher_->publish(msg);
    }

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStatus>::SharedPtr
        publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<bool, 3> ledState_{false, false, false};
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}