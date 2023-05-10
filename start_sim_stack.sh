#!/bin/bash

cd ~/Documents/Github/PX4-Autopilot/
HEADLESS=1 make px4_sitl gz_x500 &

source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args --param fcu_url:="udp://:14540@127.0.0.1:14557" --param gcs_url:="udp://@127.0.0.1:14558" --param plugin_denylist:="[odometry,image_pub,vibration,distance_sensor,rangefinder,wheel_odometry]" &

cd ~/Downloads
./QGroundControl.AppImage &