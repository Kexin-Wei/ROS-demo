#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import sys


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = position_x
    p.pose.position.y = position_y
    p.pose.position.z = 0.0
    p.pose.orientation.x = q_x
    p.pose.orientation.y = q_y
    p.pose.orientation.z = q_z
    p.pose.orientation.w = q_w
    return p


def main(init_x: float = 0.0, init_y: float = 0.0, init_rot: float = 0.0):
    # --- init
    rclpy.init()
    nav = BasicNavigator()

    # -- set initial pose
    initial_pose = create_pose_stamped(nav, init_x, init_y, 0.0)
    nav.setInitialPose(initial_pose)

    # -- wait for nav2
    nav.waitUntilNav2Active()

    # -- send commends
    points = []
    way_points = [
        (3.0, 0.0, 0),
        (3.0, 3.0, -1.57),
        (1.0, 3.5, -3.14),
        (-2.0, 3.0, 1.57),
        (0.0, 0.0, 0.0),
    ]
    for wp in way_points:
        points.append(create_pose_stamped(nav, wp[0], wp[1], wp[2]))

    nav.followWaypoints(points)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    # -- shut down
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    else:
        main(0.0, 0.0, 0.0)
