#!/bin/bash

set -e

echo "This script will configure your system"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

PACKAGE_PATH=/home/believer/catkin_ws/src/terrain-navigation

# Configure systemd service
echo "Copy systemd service"
cp -vf $PACKAGE_PATH/systemd/terrain-planner.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/mavlink-router.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/rosbag-record.service /etc/systemd/system/

systemctl enable terrain-planner.service
systemctl enable mavlink-router.service
