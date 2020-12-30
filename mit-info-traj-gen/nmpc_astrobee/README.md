# nmpc_astrobee

**MATLAB NMPC is defined in the `scripts/` subfolder!**

This ROS package has an ACADO-exported NMPC contained in `NMPC_export/` and a main.cpp to run it. The compiled product executable is a custom planner that accounts for parameter information incentivization that replaces the default Astrobee planner/controller for testing.

The NMPC uses the rigid body dynamics, and has adaptable parameters. The nodelet created by this package effectively
replaces the default `ctl` nodelet, which normally resides in `gnc/ctl/`. The newest mass and inertia values are grabbed from topic /mob/inertia.

## Products

`acado_planner` : The interface to call information-incentivizaed receding horizion trajectory generation.

`acado_caller` : The lower-level controller that handles timing issues when sending setpoints to the mixer.

## Usage

The scripts in test_session_tools are intended to run `acado_caller` and `acado_planner`, but they may also be called standalone.

After launching the simulation and reseting bias if necessary
Launch the node using  `rosrun nmpc_astrobee acado_caller` and/or `rosrun nmpc_astrobee acado_planner` 

TODO: documentation in progress.

