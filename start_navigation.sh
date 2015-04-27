#!/bin/bash

export ROS_MASTER_URI=http://dagobert:11311/

echo "-----------------------------------------------------------"
echo "-                  Start ROSCORE                          -"
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
rosrun gmapping slam_gmapping _xmin:=-10.0 _xmax:=10.0 _ymin:=-10.0 _ymax:=10.0 _delta:=0.02 _maxUrange:=5 &
PID4="$!"
sleep 4

echo "-----------------------------------------------------------"
echo "-                 Starting transforms                     -"
echo "-----------------------------------------------------------"
rosrun tf static_transform_publisher  0.034 0.0 0.250 0.0 0.0 0.0 /base_link /laser 100 &
rosrun dynamic_mapping transform_broadcaster &
PID5="$!"


echo "-----------------------------------------------------------"
echo "-                 Starting Security                       -"
echo "-----------------------------------------------------------"
rosrun p3at security_navigation #_use_laser:=1 &
PID6="$!"

echo "-----------------------------------------------------------"
echo "-                 Starting P3AT-URDF                      -"
echo "-----------------------------------------------------------"
roslaunch p2os_urdf pioneer3at_urdf.launch &
PID7="$!"
sleep 4

trap "kill $PID1 $PID2 $PID3 $PID4 $PID5 $PID6 $PID7" exit INT TERM

wait
