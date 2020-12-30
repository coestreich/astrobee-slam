/**
 * @file BlobTracker.h
 * @brief Object for tracking blobs inside 3D point clouds.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_BLOBTRACKER_H_
#define MIT_SLAM_BLOBTRACKER_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <utility>
#include <vector>
#include <random>

namespace slam {

/// 3D Point cloud-based blob tracker class.
/*!
  Astrobee's HazCam produces 3D point clouds that can be used for state
  estimation. When observing an unknown tumbling target, it is advantageous to
  try and obtain direct measurements to its center of mass. This class offers
  geometric methods to extract such information.
 */
class BlobTracker {
 public:
  /// Structure for bundling the `BlobTracker`s parameters.
  struct Params {
    /// Maximum distance for a point cloud point to be considered [m].
    float max_x_distance;
    float max_y_distance;
    float max_z_distance;
    /// Max point cloud size
    int max_pcd_size;
  };

  /// Internal copy of the parameters.
  Params params_;

  /// Constructor needs to explicitly choose whether to enable ROS components.
  explicit BlobTracker(const Params &params);
  /// Default destructor.
  ~BlobTracker();

  /// Calculates the centroid from an incoming PointCloud2 message.
  Eigen::Vector3f Centroid(const sensor_msgs::PointCloud2 &pcd);
  /// Overload to include a truncation distance for discarding points past it.
  Eigen::Vector3f TruncatedCentroid(const sensor_msgs::PointCloud2 &pcd);
  /// Calculates the centroid and its corresponding point cloud vector.
  std::pair<Eigen::Vector3f, Eigen::MatrixXf> TruncatedCentroidAndCloud(
      const sensor_msgs::PointCloud2 &pcd);
  /// Downsamples point cloud if it's too large
  Eigen::MatrixXf DownSamplePcd(const Eigen::MatrixXf &raw_pcd);

 private:
  /// The most recent point cloud centroid estimate.
  Eigen::Vector3f centroid_{0.0, 0.0, 0.0};
};

}  // namespace slam

#endif  // MIT_SLAM_BLOBTRACKER_H_
