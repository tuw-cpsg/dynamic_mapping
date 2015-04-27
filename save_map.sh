#!/bin/bash

export ROS_MASTER_URI=http://dagobert:11311/
cd ./maps/map/new
echo "-----------------------------------------------------------"
echo "-                 Saving map                              -"
echo "-----------------------------------------------------------"
rosrun dynamic_mapping map_export -f exported
#ristretto exported.ppm &

