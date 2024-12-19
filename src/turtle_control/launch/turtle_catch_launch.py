from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    turtle_spawner_node = Node(
        package="turtle_control",
        executable="turtle_spawner",
    )
    turtle_controller_node = Node(
        package="turtle_control",
        executable="turtle_controller",
    )

    return LaunchDescription(
        [turtlesim_node, turtle_spawner_node, turtle_controller_node]
    )
