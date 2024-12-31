#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class AddTwoIntsClient : public rclcpp::Node {
   public:
    AddTwoIntsClient() : Node("add_two_ints_client") {
        RCLCPP_INFO(this->get_logger(), "AddTwoIntsClient is estabished");

        thread1_ = std::thread(
            std::bind(&AddTwoIntsClient::callAddTwoIntsServer, this, 10, 4));

        thread2_ = std::thread(
            std::bind(&AddTwoIntsClient::callAddTwoIntsServer, this, 3, 9));
    }

   private:
    void callAddTwoIntsServer(int a, int b) {

        auto client = this->create_client<example_interfaces::srv::AddTwoInts>(
            "add_two_ints");
        RCLCPP_INFO(this->get_logger(), "Into callback");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for the server to be up...");
        }

        RCLCPP_INFO(this->get_logger(), "Into callback");
        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto future = client->async_send_request(request);

        try {

            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b,
                        response->sum);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error");
        }
    }

    std::thread thread1_;
    std::thread thread2_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}