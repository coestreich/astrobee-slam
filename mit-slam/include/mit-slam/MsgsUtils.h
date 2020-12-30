/**
 * @file MsgsUtils.h
 * @brief Common useful functions for ROS messages.
 * @date Apr 15, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_MSGSUTILS_H_
#define MIT_SLAM_MSGSUTILS_H_

#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <vector>

namespace slam {

/// Create the skeleton of a ROS PointCloud2 variable with pre-allocated memory
/// for the designated number of points `num_points`.
sensor_msgs::PointCloud2 PreparePointCloud2(const unsigned int num_points);

/// Create a ROS PointCloud2 from a vector of Eigen points.
sensor_msgs::PointCloud2 CreatePointCloud2(
    const std::vector<Eigen::Vector3f> &pcd);

/// Create a ROS PointCloud2 from an Eigen matrix.
sensor_msgs::PointCloud2 CreatePointCloud2(const Eigen::MatrixXf &pcd);

}  // namespace slam

#endif  // MIT_SLAM_MSGSUTILS_H_
