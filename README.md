Notes for udemy class https://www.udemy.com/course/ros2-for-beginners/?couponCode=KEEPLEARNING

# Projects
## Turtle catch game
- package `turtle_control`

![](/rm_resources/turtle_catch.gif)
## Arm car moving
- package `arm_bringup`
- with commands
    ```bash
    # terminal 1
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.01}}"
    # terminal 2
    ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {frame_id: base_footprint}, joint_names: [base_forearm_joint, forearm_hand_joint], points: [{positions: [0.5,0.3]}]}"
    ```

![](/rm_resources/arm_car.gif)
