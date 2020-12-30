# VNAV project: State Estimation of an Uncooperative, Tumbling Space Object

Implementation of SLAM and pose graph optimization for estimating the relative state and inertial parameters of an uncooperative, tumbling space object. Relative state and parameter estimation is critical for autonomous vehicles in operating in remote environments. This project uses NASA's Astrobee simulator (https://github.com/nasa/astrobee) as an experimental environment. A quick overview of the code's functionality (along with currently unresolved questions) is in `overview.txt`.

# Running the code
1. Terminal #1: `roscore` (start ROS)
2. Terminal #2: `rosparam set /use_sim_time true`, then `rviz -d rviz/point_cloud_view.rviz` (start rviz)
3. Terminal #3: `rosrun mit-slam slam_node` (start SLAM node)
4. Terminal #4: `rosbag record /mitslam/delta_pose_reg /mitslam/target_centroid /mitslam/target_points` (record SLAM data, along with others in the future)
5. Terminal #5: `rosbag play --clock <SIM_DATA_BAGFILE>` (start simulated tumbling trajectory)

# TODOs

General status: Front-end pretty much done, back-end largely untouched.

Potential way of progressing on front-end and back-end simultaneously:
1. Use simulator point cloud data to figure out front-end, have the simulator truth to compare to (pose measurements should be somewhat close).
2. For back-end, corrupt simulator ground truth poses with noise as mock front-end measurements, compare estimates with simulator ground truth state trajectory.

General items
1. Continue to develop rviz: plot estimated poses
2. Calculate ground truth state values for the trajectory
3. Record estimated states, ground truth, point clouds, etc., analyze, and write the paper.

Front-end (pretty much complete)
1. Match between non-subsequent frames
2. Eventually add some more noise to simulated point clouds, testing robustness

Back-end
1. Finalize suitable factor graph framework using point cloud odometry measurements, dynamic factors, etc.
2. Implement factor graph with GTSAM
3. Tune factor graph optimization
4. Publish final state estimates, analyze results


# Prerequisites:
### ROS melodic
### GTSAM (for factor graphs)
### OpenCV (for image visualization)
### Teaser++ (for point cloud registration and relative pose estimation)
### PCL (for point cloud feature detection and matching)

# Code Layout:
### mit-slam
The main code directory. Builds off of existing code that is attributed to Tonio Teran (MIT SSL alum). The main SLAM class is in `src/SlamNode.cpp`, and the SLAM ROS executable is in `slam_node.cpp`. Currently, all of the code is for the SLAM front-end. The relative pose measurements between nodes on the SLAM graph are obtained via point cloud registration. Point cloud registration is performed using the Teaser++ repository (see below).

### data
Includes a link to the bag file, which has the following ROS topics recorded from a three-minute tumble in the Astrobee simulator:
1. `/gnc/ekf`, the target's estimated state in the world frame.
2. `/honey/gnc/ekf`, the chaser's estimated state in the world frame.
3. `/honey/hw/depth_haz/points`, the chaser's point cloud data (from Astrobee's HazCam).
4. `/honey/hw/cam_nav`, the chaser's 2-D image data (from Astrobee's NavCam).
5. `/loc/truth/pose`, the target's ground-truth pose in the world frame.
6. `/loc/truth/twist`, the target's ground-truth pose in the world frame.
7. `/honey/loc/truth/pose`, the chaser's ground-truth pose in the world frame.
8. `/honey/loc/truth/twist`, the chaser's ground-truth pose in the world frame.

To view the stream of NavCam images, first play the rosbag in a terminal. Then, in another terminal, run the following command:
`rosrun image_view image_view image:=/honey/hw/cam_nav`.

A directory is also included of NavCam images from the tumble (purely for visualization, not used with ROS).

### external
The SLAM system relies on Teaser++ for 3D point cloud registration. It is necessary to clone this separately into the external directory. It can be found at the following link:
1. Teaser++: https://github.com/MIT-SPARK/TEASER-plusplus

### ff_msgs
This is copied from NASA's Astrobee flight software, and defines all the custom ROS messages Astrobee uses. Currently, the only necessary custom message is the EKF state (`/gnc/ekf`).

# Building the code:
1. Clone this repo into the `vnav_ws/src` directory (working in this workspace will cover the ROS, GTSAM and OpenCV dependencies).
2. Clone the Teaser++ repo into the external directory.
3. Build the code normally via `catkin build` in the `vnav_ws`.

Note: you may need to install METIS via the following command:

`sudo apt-get install libmetis-dev`

