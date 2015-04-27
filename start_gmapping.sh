#!/bin/bash

export ROS_MASTER_URI=http://dagobert:11311/

echo "-----------------------------------------------------------"
echo "-                 Starting roscore                        -"
echo "-----------------------------------------------------------"
roscore &
PID1="$!"
sleep 5
echo "-----------------------------------------------------------"
echo "-                 Starting RosAria                        -"
echo "-----------------------------------------------------------"
rosrun ROSARIA RosAria _port:=/dev/ttyS0 > /dev/null &
PID2="$!"
sleep 3
echo "-----------------------------------------------------------"
echo "-                 Starting Laser Scanner                  -"
echo "-----------------------------------------------------------"
rosrun LMS1xx LMS100 &
PID3="$!"
sleep 2
echo "-----------------------------------------------------------"
echo "-                 Starting SLAM                           -"
echo "-----------------------------------------------------------"
#rosrun gmapping slam_gmapping _xmin:=-20.0 _xmax:=20.0 _ymin:=-20.0 _ymax:=20.0 _delta:=0.025 &
rosrun gmapping slam_gmapping _xmin:=-5.0 _xmax:=45.0 _ymin:=-45.0 _ymax:=5.0 _delta:=0.025 &
PID4="$!"

echo "-----------------------------------------------------------"
echo "-                 Starting transform                      -"
echo "-----------------------------------------------------------"

rosrun tf static_transform_publisher 0.034 0.0 0.250 0.0 0.0 0.0 /base_link /laser 100 &
PID5="$!"

echo "-----------------------------------------------------------"
echo "-                 Starting Security                       -"
echo "-----------------------------------------------------------"
rosrun p3at security &
PID6="$!"

echo "-----------------------------------------------------------"
echo "-                 Starting P3AT-URDF                      -"
echo "-----------------------------------------------------------"
roslaunch p2os_urdf pioneer3at_urdf.launch 
PID7="$!"


trap "kill $PID1 $PID2 $PID3 $PID4 $PID5 $PID6 $PID7" exit INT TERM

wait
