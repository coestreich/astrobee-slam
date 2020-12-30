# inertia_estimator

This ROS package uses a linear least squares method for inertial parameter estimation of Astrobee, by Monica Ekal. It is used in conjunction with a real-time NMPC for information-incentivized reference trajectory generation.

Note: The EKF does *not* currently account for center of mass offset!

It currently has a linear least squares implementation for calculating the mass. It takes in commanded forces from `gnc/ctl/command/forces` and uses accelerations from `gnc/ctl/command/accel` (to be replaced with acceleration values calculated by differentiating *measured* velocities.)

This implementation will be incrementally built upon.

# Usage
Launch this node using  `rosrun inertia_estimator inertia_estimator`.

This node can also be launched using the launch file in `test_session_tools`.


