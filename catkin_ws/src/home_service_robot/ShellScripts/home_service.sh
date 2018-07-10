#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/nvidia/catkin_ws/src/home_service_robot/World/my_home_environment.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/nvidia/catkin_ws/src/home_service_robot/World/map.yaml" &
sleep 5
xterm  -e  " rosrun rviz rviz -d /home/nvidia/catkin_ws/src/home_service_robot/RvizConfig/home_service.rviz" & 
sleep 5
xterm  -e  " rosrun home_service_robot pick_objects" &
sleep 5
xterm  -e  " rosrun home_service_robot add_markers_final"
