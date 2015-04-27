#!/bin/bash

export ROS_MASTER_URI=http://dagobert:11311/

echo "-----------------------------------------------------------"
echo "-                 Starting keyboard_control               -"
echo "-----------------------------------------------------------"
rosrun p3at keyboard_control
PID1="$!"

trap "kill $PID1" exit INT TERM

