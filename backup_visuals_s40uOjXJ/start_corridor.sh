#!/usr/bin/env bash
set -e
corridor_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$corridor_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
cd "$corridor_ws"
exec ros2 launch uas_sim multi_drone.launch.py
