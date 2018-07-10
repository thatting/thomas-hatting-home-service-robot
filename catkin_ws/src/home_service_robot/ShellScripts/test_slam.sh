#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/nvidia/catkin_ws/src/home_service_robot/World/my_home_environment.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"


