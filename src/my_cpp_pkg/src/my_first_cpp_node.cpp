#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
   public:
    MyNode() : Node("hello_world") {
        RCLCPP_INFO(this->get_logger(), "Hello World!");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MyNode::timerCallBack, this));
    }

   private:
    void timerCallBack() {
        RCLCPP_INFO(this->get_logger(), "Hello Again %d", ++count_);
    }

    int count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}