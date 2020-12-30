# SLAM Module

Module for simultaneous localization and mapping tasks, as well as the tasks
involving the physical characterization of the target object.

## Quick Sanity Check

To make sure the package has been correctly built and registered, you can try
running a quick checkout test by using the following command:

```bash
roslaunch mit-slam quick_checkout.launch
```

A chatter node should appear, with the following output:

```bash
[ INFO] [1585775266.870438619]: Quick checkout test. Ping #0
[ INFO] [1585775266.970333283]: Quick checkout test. Ping #1
[ INFO] [1585775267.070379005]: Quick checkout test. Ping #2
[ INFO] [1585775267.170329879]: Quick checkout test. Ping #3
.
.
.
```

## Available Sensor Suite

> (From Charles and Keenan) I think the four main sensors are:

- NavCam (forward facing visual camera, `/hw/cam_nav`)
- DockCam (backward facing visual camera, `/hw/cam_dock`)
- HazCam (forward facing depth camera, `/hw/depth_haz/points`)
- PerchCam (backward facing depth camera, `/hw/depth_perch/points`)

### NavCam Properties

 Some info available
[here](https://github.com/nasa/astrobee/tree/master/simulation#under-the-hood).

The simulation's camera configurations (e.g., resolution) seem to be
here:`./description/description/urdf/sensor_nav_cam.urdf.xacro`.

Camera calibration files and properties for both hardware and I think the
simulator are located in `astrobee/config/robots`. The `sim.config` seems to be
the one being used for the simulation cameras?

To toggle the camera on and off, and to customize the frame rate, we need to
modify the values here: `./astrobee/config/simulation/simulation.config`.


# BlobTracker

A `BlobTracker` node can be used to get estimates to the target's center of
mass. The variables are part of the configuration properties that can be chosen
for the `BlobTracker`, which are located in its [yaml
file](/params/blob_params.yaml):
- `node_loop_rate` [Hz]
- `max_point_distance` [m]
- `pcd_topic`, default is `/honey/hw/depth_haz/points`
- `centroid_pub_topic`, default is `/mitslam/BlobTracker/centroid`

## Important details!

Massive slowdown when computing the point cloud corresponding to the centroid's
blob (the one obtained after thresholding by the maximum distance). Right now
it's being built from a `std::vector` of `Eigen::Vector3f`s after exiting the
centroid's loop and calling the helper function `CreatePointCloud2`. Since it's
for debugging purposed only, it is OK to do for now, but I should remove this
for the release version after making sure everything works.

