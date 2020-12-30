## Inputs

Current phase of operation.

* Param: /td/est_phase : {'off', 'initial_survey', 'continuous_survey'}


## Outputs

Parameters:

* inertia_ratios ({tonio writes these here})
* +covariances
* msg: `InertialParams.msg`, topic: `/honey/mit_slam/inertial_params`

State (+Whole Pose):

* Target pose R_TI with respect to INERTIAL frame (position of COM, orientation of principal axes), (geometry_msgs::PoseStamped))
* Target angular velocity in the Target BODY frame(geometry_msgs::PointStamped))
* +covariances
* msg: `AngState.msg`, topic: `/honey/mit_slam/ang_state`

Message files for inertial parameters and states can be found in `mit-slam/msg`.

## TODO
[ ] SLAM backend
[ ] interface with Tonio's node(s), "slam_node"
[ ] integrate estimation into dry run
[ ] test cross-compile with TEASER and GTSAM 
[ ] debug forever
