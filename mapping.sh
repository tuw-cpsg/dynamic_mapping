#!/bin/bash

 # only for home office
#export ROS_MASTER_URI=http://localhost:11311/
#export ROS_IP=127.0.0.1
 # for lab
export ROS_IP=10.42.0.19
export ROS_MASTER_URI=http://dagobert:11311/

#cd ./maps

#echo "-----------------------------------------------------------"
#echo "-                  Start ROSCORE                          -"
#echo "-----------------------------------------------------------"
#roscore &
#PID1="$!"
#sleep 3

#echo "-----------------------------------------------------------"
#echo "-   (home office only) Publish faked SLAM Map /slam_map   -"
#echo "-----------------------------------------------------------"
#rosrun map_server map_server ./16.08.2013/map_grey.yaml &
#PID2="$!"


echo "-----------------------------------------------------------"
echo "-        Starting dynamic map import from file            -"
echo "-----------------------------------------------------------"
#rosrun dynamic_mapping main exported_map2.header  #dynamic_mapping import
rosrun dynamic_mapping main #exported_map.header
PID3="$!"
sleep 3

#echo "-----------------------------------------------------------"
#echo "-             Starting Dynamic mapping export             -"
#echo "-----------------------------------------------------------"
#rosrun dynamic_mapping map_export -f exported_map #dynamic_mapping export
#PID3="$!"




trap "kill $PID1 $PID2 $PID3" exit INT TERM

wait
