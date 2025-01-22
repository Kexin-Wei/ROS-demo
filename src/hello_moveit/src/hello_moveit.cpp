#include <cstdio>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char** argv) {
    // init
    rclcpp::init(argc, argv);

    // create node
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    auto const logger = rclcpp::get_logger("hello_moveit");
    RCLCPP_INFO(logger, "Hello MoveIt!");

    // create the move group interface
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // set a target pose
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = .28;
        msg.position.y = -.2;
        msg.position.z = .5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // create a plan
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_tuple(ok, msg);
    }();

    // execute the plan
    if (success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_DEBUG(logger, "Failed to plan");
    }
    // shutdown
    rclcpp::shutdown();
    return 0;
}
