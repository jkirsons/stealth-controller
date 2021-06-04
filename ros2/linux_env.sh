#!/bin/bash
set -v
cd ~/dev_ws
source /opt/ros/galactic/setup.bash
. install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.1.26:11811
set +v