#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/model/drone_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_4/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_4/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_5/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_5/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_6/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_6/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_7/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_7/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/model/drone_8/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/drone_8/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
