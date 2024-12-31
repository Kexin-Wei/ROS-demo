#include <chrono>
#include <mutex>

#include "my_robot_interfaces/msg/led_status.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

class TurtleSpawnerNode : public rclcpp::Node {
   public:
    TurtleSpawnerNode() : Node("led_panel_node") {
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led", std::bind(&TurtleSpawnerNode::callbackSetLed, this,
                                 std::placeholders::_1, std::placeholders::_2));
        publisher_ =
            this->create_publisher<my_robot_interfaces::msg::LedStatus>(
                "led_panel_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleSpawnerNode::publishLedStatusCallback, this));

        // set parameter
        this->declare_parameter("led_state",
                                std::vector<bool>{true, true, true});
        auto copy_ledState = [this](const std::vector<bool>& source) {
            this->ledState_.resize(source.size());
            this->ledState_.assign(source.begin(), source.end());
        };
        auto param = this->get_parameter("led_state").as_bool_array();
        copy_ledState(param);
        // set param callback has unknow error related to memory, too hard to solve
        // even lock can't help, or maybe use lock in the wrong way
        // auto on_set_led_state_callback =
        //     [this,
        //      &copy_ledState](const std::vector<rclcpp::Parameter>& parameters) {
        //         const std::lock_guard<std::mutex> lock(this->ledState_mutex_);
        //         rcl_interfaces::msg::SetParametersResult result;
        //         result.successful = true;
        //         for (const auto p : parameters) {
        //             if (p.get_name() == "led_state") {
        //                 RCLCPP_INFO(this->get_logger(), "parameter updated");
        //                 auto param = p.as_bool_array();
        //                 copy_ledState(param);
        //                 RCLCPP_INFO(this->get_logger(), "Succeed updated");
        //             }
        //         }
        //         return result;
        //     };
        // on_set_handle_ =
        //     this->add_on_set_parameters_callback(on_set_led_state_callback);
        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    void callbackSetLed(
        const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response) {
        auto idx = request->led_index;
        if (idx >= int(ledState_.size()) || idx < 0) {
            response->success = false;
            response->message = "No such a led, out of index.";
            return;
        }
        ledState_[idx] = request->set_state;
        response->success = true;
        response->message = "Set done";
        RCLCPP_INFO(this->get_logger(), "%ld set to %s", idx,
                    request->set_state ? "true" : "fasle");
        publishLedStatusCallback();
    }

    void publishLedStatusCallback() {
        auto msg = my_robot_interfaces::msg::LedStatus();
        for (const auto s : ledState_) {
            msg.led_states.push_back(s);
        }
        publisher_->publish(msg);
    }

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStatus>::SharedPtr
        publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_handle_;
    std::vector<bool> ledState_;
    std::mutex ledState_mutex_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}