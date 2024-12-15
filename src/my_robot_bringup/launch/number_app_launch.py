from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    remap_number_topic = ("number", "number_new")

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="number_publisher_new",
        remappings=[remap_number_topic],
        parameters=[{"number": 5}],
    )

    counter_node = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        remappings=[
            remap_number_topic,
            ("number_count", "number_count_new"),
        ],
    )
    ld.add_action(number_publisher_node)
    ld.add_action(counter_node)
    return ld
