#!/bin/bash
# ROSbag recorder for the pose of the target and the chaser.

# Source the Astrobee ROS directory.
source /home/tars/repos/tumbledock/freeflyer-build/devel/setup.bash

TOPICS="/honey/gnc/ekf /honey/loc/truth/pose /honey/loc/truth/twist\
        /gnc/ekf /loc/truth/pose /loc/truth/twist"

# Start recording the data.
rosbag record -o poses ${TOPICS}
       # /honey/gnc/ekf\         # EKF output for the chaser-pcd_2020-05-06-20-29-11.bag.
       # /honey/loc/truth/pose\  # Gazebo position and attitude ground truth.
       # /honey/loc/truth/twist\ # Gazebo linear and angular vel ground truth.
       # /gnc/ekf\               # EKF output for the target.
       # /loc/truth/pose\        # Gazebo position and attitude ground truth.
       # /loc/truth/twist        # Gazebo linear and angular vel ground truth.
