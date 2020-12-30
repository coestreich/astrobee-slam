/**
 * @file slam_node.cpp
 * @brief Main SLAM system executable.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/SlamNode.h"
#include "mit-slam/BlobTracker.h"
#include "mit-slam/CloudOdometer.h"
#include "mit-slam/GraphManager.h"
#include "mit-slam/ParamUtils.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_node");

  // Instantiate the SLAM system.
  slam::SlamNode::Params slam_params = slam::SlamParamsFromRos();
  slam::SlamNode slam_node{slam_params};
  ros::Rate loop_rate(slam_params.loop_rate);

  // Activate the blob tracker.
  slam::BlobTracker::Params blob_params = slam::BlobParamsFromRos();
  slam_node.SetupBlobTracker(blob_params);

  // Activate the cloud odometer.
  slam::CloudOdometer::Params cloud_params = slam::CloudParamsFromRos();
  slam_node.SetupCloudOdometer(cloud_params);

  // Activate the graph manager
  slam::GraphManager::Params graph_params = slam::GraphParamsFromRos();
  slam_node.SetupGraphManager(graph_params);

  // Run the node
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
