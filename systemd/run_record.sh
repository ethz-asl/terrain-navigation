#!/bin/bash

set -e


gst-launch-1.0 -e rtspsrc location=rtsp://127.0.0.1:8553/stream protocols=tcp ! rtph264depay ! h264parse ! mp4mux ! filesink location=~/rosbag/onboard_$(date +%s).mp4&

source $HOME/catkin_ws/devel/setup.bash
roslaunch terrain_navigation_ros run_rosbag.launch
