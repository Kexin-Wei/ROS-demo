from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    list_of_node = []
    n_robot_news_station = 4
    for i in range(4):
        list_of_node.append(
            Node(
                package="my_py_pkg",
                executable="robot_news_station",
                name=f"robot_news_station_{i}",
                parameters=[{"robot_name": f"test_{i}"}],
            )
        )
    list_of_node.append(
        Node(
            package="my_py_pkg",
            executable="a_listener",
        )
    )

    return LaunchDescription(list_of_node)
