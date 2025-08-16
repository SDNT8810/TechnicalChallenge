#!/bin/bash

clear

echo "Running the Technical Challenge..."

rm -r -f build log install

pkill -9 gzserver
pkill -9 gzclient

pkill -f "ros2 launch\|gazebo\|gzserver\|gzclient\|rviz2\|rviz"

colcon build

. install/setup.bash

ros2 launch sdnt_robot_description display.launch.py

# ros2 launch tb3_nav2 simulation.launch.py 
