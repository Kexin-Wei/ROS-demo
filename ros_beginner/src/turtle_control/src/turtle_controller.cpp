#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_relative.hpp"

#include "turtle_control/msg/caught_it.hpp"
#include "turtle_control/msg/goal.hpp"
#include "turtle_control/srv/kill_first.hpp"

class TurtleControllerNode : public rclcpp::Node {
   public:
    TurtleControllerNode() : Node("turtle_controller") {
        // add subscriber to get goal
        subscriberGoal_ = this->create_subscription<turtle_control::msg::Goal>(
            "goal", 10,
            std::bind(&TurtleControllerNode::callbackGetGoal, this,
                      std::placeholders::_1));

        // add subscriber to get the current pose
        subscribterCurrentPose_ =
            this->create_subscription<turtlesim::msg::Pose>(
                turtleName_ + "/pose", 10,
                std::bind(&TurtleControllerNode::callbackGetCurrentPose, this,
                          std::placeholders::_1));

        // add publisher for caught it
        publishCaughtIt_ =
            this->create_publisher<turtle_control::msg::CaughtIt>("caught_it",
                                                                  10);
        timerCaughtIt_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleControllerNode::callbackPublishCaughtIt, this));

        // move turtle
        timerMoveTurtle_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TurtleControllerNode::closedLoopControl, this));
        timerMoveTurtle_->cancel();

        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    void closedLoopControl() {
        const std::lock_guard<std::mutex> lock1_(mutexGoal_);
        const std::lock_guard<std::mutex> lock2_(mutexCurrentPose_);
        const std::lock_guard<std::mutex> lock3_(mutexCaughtIt_);

        auto dx = goal_[0] - currentPose_[0];
        auto dy = goal_[1] - currentPose_[1];
        auto dist = sqrt(dx * dx + dy * dy);
        auto distAngle = atan2(dy, dx);
        auto dthetaC = distAngle - currentPose_[2];
        auto dtheta = clampAngleInTwoPi(dthetaC);
        float dorient{0}, ddist{0};
        if (dist > 0.5) {
            if (abs(dtheta) < angleStep_)
                dorient = dtheta;
            else
                dorient = dtheta > 0 ? angleStep_ : -angleStep_;
            ddist = dist > distStep_ ? distStep_ : dist;
            caughtIt_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Caught it.");
            caughtIt_ = true;
        }

        threadsMoveTurtle_.push_back(std::make_shared<std::thread>(std::bind(
            &TurtleControllerNode::callbackMoveTurtle, this, ddist, dorient)));
        RCLCPP_DEBUG(this->get_logger(),
                     "dx %.1f, dy %.1f, ds %.1f, dsa "
                     "%.1f, dbc %.1f, da %.1f, dd %.1f, do %.1f",
                     dx, dy, dist, distAngle, dthetaC, dtheta, ddist, dorient);
    };

    void callbackKillTurtle() {
        auto client = this->create_client<turtle_control::srv::KillFirst>(
            "kill_first_tut");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(),
                        "Waiting for kill first tut server...");
        }

        auto request =
            std::make_shared<turtle_control::srv::KillFirst::Request>();
        auto future = client->async_send_request(request);
        try {
            auto respone = future.get();
            RCLCPP_INFO(this->get_logger(), "Kill turtle %s",
                        respone->success ? "succeed" : "failed");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error when kill turtle, %s",
                         e.what());
        }
    }

    void callbackPublishCaughtIt() {
        // const std::lock_guard<std::mutex> lock_(mutexCaughtIt_);
        auto msg = turtle_control::msg::CaughtIt();
        msg.caught_it = caughtIt_;
        publishCaughtIt_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Publish CaughtIt");
    }

    void callbackMoveTurtle(float ddist, float dorient) {
        auto client = this->create_client<turtlesim::srv::TeleportRelative>(
            turtleName_ + "/teleport_relative");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(),
                        "Waiting for teleport_relative server...");
        }

        auto request =
            std::make_shared<turtlesim::srv::TeleportRelative::Request>();
        request->angular = dorient;
        request->linear = ddist;
        auto future = client->async_send_request(request);
        try {
            auto response = future.get();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error when callbackMoveTurtle, %s", e.what());
        }
    }

    void callbackGetCurrentPose(const turtlesim::msg::Pose::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock_(mutexCurrentPose_);
        currentPose_[0] = msg->x;
        currentPose_[1] = msg->y;
        currentPose_[2] = msg->theta;
    }

    void callbackGetGoal(const turtle_control::msg::Goal::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock_(mutexGoal_);
        goal_[0] = msg->x;
        goal_[1] = msg->y;
        if (timerMoveTurtle_->is_canceled()) {
            timerMoveTurtle_->reset();
            RCLCPP_INFO(this->get_logger(), "Start to catch turtle");
        }
    }

    float clampAngleInTwoPi(float angle) {
        angle = fmod(angle, 2 * M_PI);
        if (angle < -M_PI)
            angle += 2 * M_PI;
        else if (angle > M_PI)
            angle -= 2 * M_PI;
        return angle;
    }

    rclcpp::Subscription<turtle_control::msg::Goal>::SharedPtr subscriberGoal_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr
        subscribterCurrentPose_;

    rclcpp::TimerBase::SharedPtr timerCaughtIt_;
    rclcpp::Publisher<turtle_control::msg::CaughtIt>::SharedPtr
        publishCaughtIt_;
    std::vector<std::shared_ptr<std::thread>> threadsCaughtIt_;

    rclcpp::TimerBase::SharedPtr timerMoveTurtle_;
    std::vector<std::shared_ptr<std::thread>> threadsMoveTurtle_;

    std::vector<std::shared_ptr<std::thread>> threadsKillTurtle_;

    std::mutex mutexCaughtIt_;
    std::mutex mutexGoal_;
    std::mutex mutexCurrentPose_;
    std::array<float, 2> goal_;
    std::array<float, 3> currentPose_;  // x,y, theta

    std::string turtleName_{"turtle1"};
    float angleStep_{0.2};
    float distStep_{0.1};
    bool caughtIt_{false};
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}