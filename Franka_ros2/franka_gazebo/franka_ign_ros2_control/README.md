## Franka Ign ROS 2 Control

This package is based on https://github.com/ros-controls/gz_ros2_control/tree/humble

One main difference is that for Franka robots in gravity compenstation is added in the hardware side.
To be consistent with our examples, we added the gravity torque term for each joint in the ign_system.cpp
