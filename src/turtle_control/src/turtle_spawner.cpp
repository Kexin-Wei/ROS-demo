#include <chrono>
#include <cstdlib>
#include <queue>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtle_control/srv/kill_first.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

struct TurtleGoalInfo {
    std::string name;
    float x;
    float y;
    float theta;
};

class TurtleSpawnerNode : public rclcpp::Node {
   public:
    TurtleSpawnerNode() : Node("turtle_container_node") {
        // add turtle every 3 second
        timerAddTurtle = this->create_wall_timer(
            std::chrono::seconds(8),
            std::bind(&TurtleSpawnerNode::callbackAddTurtle, this));
        // test kill turtle callback
        server = this->create_service<turtle_control::srv::KillFirst>(
            "kill_first_tut",
            std::bind(&TurtleSpawnerNode::callbackKillTurtle, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s is established.", this->get_name());
    }

   private:
    /************************************* Kill Turtle *********************************************/
    void callbackKillTurtle(
        const turtle_control::srv::KillFirst::Request::SharedPtr request,
        const turtle_control::srv::KillFirst::Response::SharedPtr response) {
        RCLCPP_DEBUG(this->get_logger(), "into callbackKillTurtle");

        response->success = true;
        if (turtleGoals.empty()) {
            RCLCPP_INFO(this->get_logger(), "No turtle to kill");
            return;
        }
        auto t = turtleGoals.front();
        turtleKillThreads.push_back(std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::killATurtle, this, t.name)));
        turtleGoals.pop();
        RCLCPP_INFO(this->get_logger(), "%ld turtles left", turtleGoals.size());
        RCLCPP_DEBUG(this->get_logger(), "exit callbackKillTurtle");
    }

    void killATurtle(const std::string& turtleName) {
        // go and kill the first turtle

        RCLCPP_DEBUG(this->get_logger(), "into killATurtle");
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for server...");
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
        RCLCPP_DEBUG(this->get_logger(), "into callbackAddTurtle");
        TurtleGoalInfo t;
        t.x = getRandomInRange(2, 9);
        t.y = getRandomInRange(2, 9);
        t.theta = getRandomInRange(0, 3.1415 * 2);
        t.name = "tut" + std::to_string(turtleGoals.size());

        turtleSpawnThreads.push_back(std::make_shared<std::thread>(std::bind(
            &TurtleSpawnerNode::addTurtle, this, t.x, t.y, t.theta, t.name)));
        RCLCPP_INFO(this->get_logger(), "%f %f %f %s", t.x, t.y, t.theta,
                    t.name.c_str());
        turtleGoals.push(t);
        RCLCPP_DEBUG(this->get_logger(), "exit callbackAddTurtle");
    }

    void addTurtle(float x, float y, float theta, const std::string& name) {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for server...");
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

    rclcpp::TimerBase::SharedPtr timerAddTurtle;
    rclcpp::Service<turtle_control::srv::KillFirst>::SharedPtr server;
    std::vector<std::shared_ptr<std::thread>> turtleSpawnThreads;
    std::vector<std::shared_ptr<std::thread>> turtleKillThreads;
    std::queue<TurtleGoalInfo> turtleGoals;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}