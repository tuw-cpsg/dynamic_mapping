#!/bin/bash

export ROS_MASTER_URI=http://localhost:11311/
#export ROS_IP=192.168.0.15 # only for home office

cd ./maps

echo "-----------------------------------------------------------"
echo "-                  Start ROSCORE                          -"
echo "-----------------------------------------------------------"
roscore &
PID1="$!"
sleep 5

echo "-----------------------------------------------------------"
echo "-            Starting dynamic map import                  -"
echo "-----------------------------------------------------------"
rosrun dynamic_mapping main map.header & #dynamic_mapping import
PID2="$!"
sleep 3

echo "-----------------------------------------------------------"
echo "-             Starting Dynamic mapping export             -"
echo "-----------------------------------------------------------"
rosrun dynamic_mapping map_export -f exported_map #dynamic_mapping export
PID3="$!"
sleep 5




trap "kill $PID1 $PID2 $PID3" exit INT TERM

wait
