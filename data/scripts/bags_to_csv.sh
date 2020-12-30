#!/bin/bash

rostopic echo -b moving_results_good_DEC10.bag -p /honey/hw/imu > chaser_imu_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /mitslam/deltapose > chaser_deltapose_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /mitslam/chaser/pose > chaser_est_pose_moving_good_DEC10.csv 
rostopic echo -b moving_results_good_DEC10.bag -p /mitslam/chaser/twist > chaser_est_twist_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /mitslam/target/pose > target_est_pose_moving_good_DEC10.csv 
rostopic echo -b moving_results_good_DEC10.bag -p /mitslam/target/twist > target_est_twist_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /honey/loc/truth/pose > chaser_gt_pose_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /honey/loc/truth/twist > chaser_gt_twist_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /loc/truth/pose > target_gt_pose_moving_good_DEC10.csv
rostopic echo -b moving_results_good_DEC10.bag -p /loc/truth/twist > target_gt_twist_moving_good_DEC10.csv

