# test_session_tools

This package contains high-level scripts for coordinating test sessions, which were used for Astrobee ground testing, Feb-2020.

## Usage
Launch test commanding using:

`rosrun test_session_tools run_test [test number]`

TODO: documentation in progress.

## Launch files

Additional launch files are available to automatically start nodes.

`run_nmpc.launch` : starts a traj. opt. caller, and a planner to dole out trajectory setpoints.

`run_nmpc_and_est.launch` : additionally, starts up a node for inertial parameter estimation.

