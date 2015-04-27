#!/bin/bash

export ROS_MASTER_URI=http://dagobert:11311/

echo "-----------------------------------------------------------"
echo "-                 Starting RVIZ                           -"
echo "-----------------------------------------------------------"
rosrun rviz rviz navigation.rviz #p3at.rviz
PID1="$!"

trap "kill $PID1" exit INT TERM

