#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp"

class NumberCounter : public rclcpp::Node {
   public:
    NumberCounter() : Node("number_counter") {
        publisher_ =
            this->create_publisher<std_msgs::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounter::callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NumberCounter::publishCount, this));

        server_ = this->create_service<std_srvs::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounter::resetCounterCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "NumberCounter has started");
    }

   private:
    void publishCount() {
        auto msg = std_msgs::msg::Int64();
        msg.data = count_;
        publisher_->publish(msg);
    }

    void callback(const std_msgs::msg::Int64 msg) {
        count_ += msg.data;
        RCLCPP_INFO(this->get_logger(), "Received %ld, count is %d", msg.data,
                    count_);
    }

    void resetCounterCallback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        const std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (request->data == true) {
            count_ = 0;
            RCLCPP_INFO(this->get_logger(), "Reset counter");
            response->success = true;
            response->message = "Succeed";
        } else {
            response->success = false;
            response->message = "Failed";
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}