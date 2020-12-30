#!/bin/bash
# ROSbag recorder for all chaser sensors.

# Source the Astrobee ROS directory.
source /home/tars/repos/tumbledock/freeflyer-build/devel/setup.bash

# Start recording the data.
rosbag record -o sensors-chaser \
       /honey/hw/imu\
       /honey/hw/cam_dock\
       /honey/hw/cam_nav\
       /honey/hw/depth_haz/points\
       /honey/hw/depth_perch/points\
       /tf
