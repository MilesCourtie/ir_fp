#!/usr/bin/bash

# ros setup
source /opt/ros/noetic/setup.bash; cd ~/catkin_ws; source devel/setup.bash

# query service
rosservice call '/record_path_control' 1
