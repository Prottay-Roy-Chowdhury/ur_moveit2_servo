#!/usr/bin/env zsh

set -e

# Set up the environment
source "/opt/ros/${ROS_DISTRO}/setup.zsh"
source "/dev_ws/install/setup.zsh"
source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh"
# dds
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec "$@"