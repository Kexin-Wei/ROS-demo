from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    turtle_container_node = Node(
        package="my_cpp_pkg",
        executable="turtle_container",
    )

    return LaunchDescription([turtlesim_node])
