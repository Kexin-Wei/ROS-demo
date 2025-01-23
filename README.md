Notes for udemy class https://www.udemy.com/course/ros2-for-beginners/?couponCode=KEEPLEARNING

# Projects
## Turtle catch game
- package `turtle_control` in branch `ros_beginner`

![](/rm_resources/turtle_catch.gif)

## Arm car moving
- package `arm_bringup` in branch `ros_beginner`
- with commands
    ```bash
    # terminal 1
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.01}}"
    # terminal 2
    ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {frame_id: base_footprint}, joint_names: [base_forearm_joint, forearm_hand_joint], points: [{positions: [0.5,0.3]}]}"
    ```

![](/rm_resources/arm_car.gif)

## SLAM demo
- dependency package [Nav2](https://docs.nav2.org/)
- add-on launch file from `turtlebot3_gazebo` package
- map using `maps/map_floor.yaml` generated using `gazebo`
- wallpoint running script is at `src/nav2_wallpoint.py` in package

![](/rm_resources/turtlebot_wallpoint_nav.gif)

## Robot Motion Planning (TBC)
- dependency package [Moveit](https://moveit.picknik.ai/main/index.html)
- package `hello_moveit` to move to a target pose given control group
![](/rm_resources/moveit_simple.gif)

- package `hello_moveit_draw` to draw the trajectory out in space
![](/rm_resources/moveit_trajectory.gif)

- package `hello_moveit_object` to avoid a object in space
![](/rm_resources/moveit_obstacle.gif)

- package `mtc_tutorial` to pick a object and move it in space
![](/rm_resources/moveit_pick.gif)