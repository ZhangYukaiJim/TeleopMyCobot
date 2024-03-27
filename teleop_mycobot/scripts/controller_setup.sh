#!/bin/zsh
parent_path=$( cd "$(dirname "${(%):-%N}")" ; pwd -P )
cd "$parent_path"
source /opt/ros/noetic/setup.zsh
source ../../../../devel/setup.zsh
export ROS_MASTER_URI=http://10.4.176.116:11311
export ROS_HOSTNAME=10.4.176.116