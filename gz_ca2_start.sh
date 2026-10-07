#!/bin/bash
# Same as gz_ca2.sh, but the robot's start position is given as arguments: ./gz_ca2_start.sh X Y
source install/setup.bash
ros2 launch rb2301_gz ca2_gazebo.launch.py x:=$1 y:=$2
pkill -9 ruby # kills any lingering Gazebo processes
