#!/bin/bash

export MYCOBOT_ROS_1_INIT=1
export ROS_MASTER_URI=http://10.4.176.116:11311
export ROS_HOSTNAME=10.4.159.122

# Run gst-launch-1.0 command in the background
gst-launch-1.0 -q v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! rtpjpegpay ! udpsink host=10.4.176.116 port=5000 &

# Sleep for a short duration to allow gst-launch-1.0 to start
sleep 2

# Change permissions on /dev/ttyTHS1
sudo chmod +r /dev/ttyTHS1

# Run rosrun command
rosrun mycobot_280jn_moveit sync_plan.py
