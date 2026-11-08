#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/model/drone_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
