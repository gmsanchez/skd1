#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/workspaces/skd1_ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=TCPv4:[192.168.1.100]:42100
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
