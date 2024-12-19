#include <chrono>
#include <cstdlib>
#include <queue>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

#include "turtle_control/msg/caught_it.hpp"
#include "turtle_control/msg/goal.hpp"
#include "turtle_control/srv/kill_first.hpp"

struct TurtleGoalInfo {
    TurtleGoalInfo() : name(""), x(0), y(0), theta(0) {}

    std::string name;
    float x;
    float y;
    float theta;
};

class TurtleSpawnerNode : public rclcpp::Node {
   public:
    TurtleSpawnerNode() : Node("turtle_container_node") {
        // add topic to send goal location
        publisherGoal_ =
            this->create_publisher<turtle_control::msg::Goal>("goal", 10);
        timerGoal_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleSpawnerNode::callbackSendGoal, this));

        // add turtle every 3 second
        timerAddTurtle_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TurtleSpawnerNode::callbackAddTurtle, this));
        callbackAddTurtle();

        // add to subscriber turtle caught it
        subscriberCaughtIt_ =
            this->create_subscription<turtle_control::msg::CaughtIt>(
                "caught_it", 10,
                std::bind(&TurtleSpawnerNode::callbackGetCaughtIt, this,
                          std::placeholders::_1));

        // test kill turtle callback
        // serverKillTurtle_ =
        //     this->create_service<turtle_control::srv::KillFirst>(
        //         "kill_first_tut",
        //         std::bind(&TurtleSpawnerNode::callbackKillTurtle, this,
        //                   std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    void callbackGetCaughtIt(
        const turtle_control::msg::CaughtIt::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock_(mutexGoals_);
        if (msg->caught_it && !caughtItBefore_) {
            RCLCPP_INFO(this->get_logger(), "Caught it in spawner");
            if (goals.empty()) {
                RCLCPP_INFO(this->get_logger(), "No turtle to kill");
                return;
            }
            auto t = goals.front();
            threadsKill_.push_back(std::make_shared<std::thread>(
                std::bind(&TurtleSpawnerNode::killATurtle, this, t.name)));
            goals.pop();
            RCLCPP_INFO(this->get_logger(), "kill %s, %ld turtles left",
                        t.name.c_str(), goals.size());
        }
        caughtItBefore_ = msg->caught_it;
    }

    void callbackSendGoal() {
        const std::lock_guard<std::mutex> lock_(mutexGoals_);
        auto t = goals.front();
        auto msg = turtle_control::msg::Goal();
        msg.x = t.x;
        msg.y = t.y;
        publisherGoal_->publish(msg);
    }

    /************************************* Kill Turtle *********************************************/
    void callbackKillTurtle(
        const turtle_control::srv::KillFirst::Request::SharedPtr,
        const turtle_control::srv::KillFirst::Response::SharedPtr response) {
        const std::lock_guard<std::mutex> lock_(mutexGoals_);
        RCLCPP_DEBUG(this->get_logger(), "into callbackKillTurtle");

        response->success = true;
        if (goals.empty()) {
            RCLCPP_INFO(this->get_logger(), "No turtle to kill");
            return;
        }
        auto t = goals.front();
        threadsKill_.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::killATurtle, this, t.name)));
        goals.pop();
        RCLCPP_INFO(this->get_logger(), "kill %s, %ld turtles left",
                    t.name.c_str(), goals.size());
    }

    void killATurtle(const std::string& turtleName) {
        // go and kill the first turtle

        RCLCPP_DEBUG(this->get_logger(), "into killATurtle");
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for kill server...");
        }
        RCLCPP_DEBUG(this->get_logger(), "Into sending second service");

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtleName;
        auto future = client->async_send_request(request);
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%s is killed.",
                        turtleName.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error when kill turtle, %s",
                         e.what());
        }
        RCLCPP_DEBUG(this->get_logger(), "exit killATurtle");
    };

    /****************************************** Add Turtle ****************************************/
    void callbackAddTurtle() {
        const std::lock_guard<std::mutex> lock_(mutexGoals_);
        RCLCPP_DEBUG(this->get_logger(), "into callbackAddTurtle");
        if (goals.size() >= 10)
            return;

        TurtleGoalInfo t;
        t.x = getRandomInRange(2, 9);
        t.y = getRandomInRange(2, 9);
        t.theta = getRandomInRange(0, 3.1415 * 2);
        // t.name = "tut" + std::to_string(goals.size());
        t.name = "tut" + std::to_string(numOfTut_++);

        threadsSpawn_.push_back(std::make_shared<std::thread>(std::bind(
            &TurtleSpawnerNode::addTurtle, this, t.x, t.y, t.theta, t.name)));
        RCLCPP_INFO(this->get_logger(), "%f %f %f %s", t.x, t.y, t.theta,
                    t.name.c_str());
        goals.push(t);
        RCLCPP_DEBUG(this->get_logger(), "exit callbackAddTurtle");
    }

    void addTurtle(float x, float y, float theta, const std::string& name) {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for spawn server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;
        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle %s added",
                        response->name.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error with adding turtle, %s",
                         e.what());
        }
        RCLCPP_DEBUG(this->get_logger(), "exit addTurtle");
    }

    float getRandomInRange(float min, float max) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> distrib(min, max);

        // Generate a single random float
        float random_float = distrib(gen);
        return random_float;
    }

    rclcpp::TimerBase::SharedPtr timerAddTurtle_;
    rclcpp::Service<turtle_control::srv::KillFirst>::SharedPtr
        serverKillTurtle_;
    rclcpp::TimerBase::SharedPtr timerGoal_;
    rclcpp::Publisher<turtle_control::msg::Goal>::SharedPtr publisherGoal_;
    rclcpp::Subscription<turtle_control::msg::CaughtIt>::SharedPtr
        subscriberCaughtIt_;

    std::vector<std::shared_ptr<std::thread>> threadsSpawn_;
    std::vector<std::shared_ptr<std::thread>> threadsKill_;

    std::queue<TurtleGoalInfo> goals;
    std::mutex mutexGoals_;

    bool caughtItBefore_{false};
    int numOfTut_{0};
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}