#include <cstdio>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // init
    rclcpp::init(argc, argv);

    // create node
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit_draw",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    auto const logger = rclcpp::get_logger("hello_moveit_draw");
    RCLCPP_INFO(logger, "Hello MoveIt!");

    // spin a singleThreadedExecutor so that MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // create the move group interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // create a closure for visualization
    auto const draw_title = [&moveit_visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text,
                                        rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };

    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };

    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group_interface.getRobotModel()->getJointModelGroup(
             "panda_arm")](auto const trajectory) {
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

    // create collision object for the robot to avoid
    auto const collision_object =
        [frame_id = move_group_interface.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = frame_id;
            collision_object.id = "box1";
            shape_msgs::msg::SolidPrimitive primitive;

            // define the size of the box in meters
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.5;
            primitive.dimensions[primitive.BOX_Y] = 0.1;
            primitive.dimensions[primitive.BOX_Z] = 0.5;

            //define the pose of the box (relative to frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.2;
            box_pose.position.y = 0.2;
            box_pose.position.z = 0.25;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);

            collision_object.operation = collision_object.ADD;
            return collision_object;
        }();

    // add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects({collision_object});

    // set a target pose
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = .28;
        msg.position.y = .4;
        msg.position.z = .5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // create a plan
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();

    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_tuple(ok, msg);
    }();

    // execute the plan
    if (success) {
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        move_group_interface.execute(plan);
    } else {
        draw_title("Failed to plan");
        moveit_visual_tools.trigger();
        RCLCPP_DEBUG(logger, "Failed to plan");
    }

    // shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
