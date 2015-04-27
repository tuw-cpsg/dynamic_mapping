#!/bin/bash
cd ..
catkin_make -DCATKIN_BLACKLIST_PACKAGES="p2os_driver;p2os_launch;p2os_teleop;p2os_urdf;gscam"

