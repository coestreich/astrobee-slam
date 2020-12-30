/**
 * @file BlobTracker.cpp
 * @brief Object for tracking blobs inside 3D point clouds.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/BlobTracker.h"

namespace slam {

BlobTracker::BlobTracker(const BlobTracker::Params& params) : params_(params) {}

BlobTracker::~BlobTracker() {}

Eigen::Vector3f BlobTracker::Centroid(const sensor_msgs::PointCloud2& pcd) {
  // Variables to track centroid, pointer to data, and useful data points.
  Eigen::Vector3f centroid{0.0, 0.0, 0.0};
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcd, "x");
  int inliers = 0, total = 0;

  // Loop over all points to get the centroid [m].
  for (; iter_x != iter_x.end(); ++iter_x) {
    Eigen::Vector3f point{iter_x[0], iter_x[1], iter_x[2]};
    // Discard points that are all zero.
    if (point.norm() != 0) {
      // Add information to centroid.
      centroid += point;
      inliers++;
    }
    total++;
  }
  // Return normalized centroid.
  return centroid / inliers;
}

Eigen::Vector3f BlobTracker::TruncatedCentroid(
    const sensor_msgs::PointCloud2& pcd) {
  // Variables to track centroid, pointer to data, and useful data points.
  Eigen::Vector3f centroid{0.0, 0.0, 0.0};
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcd, "x");
  int inliers = 0, total = 0;

  // Loop over all points to get the centroid [m].
  for (; iter_x != iter_x.end(); ++iter_x) {
    Eigen::Vector3f point{iter_x[0], iter_x[1], iter_x[2]};
    // Discard points that are all zero, or too far away.
    float distance = point.norm();
    if (distance != 0 && (std::abs(point(0)) <= params_.max_x_distance &&
                          std::abs(point(1)) <= params_.max_y_distance &&
                          std::abs(point(2)) <= params_.max_z_distance)) {
      // Add information to centroid.
      centroid += point;
      inliers++;
    }
    total++;
  }
  // Return normalized centroid.
  return centroid / inliers;
}

std::pair<Eigen::Vector3f, Eigen::MatrixXf>
BlobTracker::TruncatedCentroidAndCloud(const sensor_msgs::PointCloud2& pcd) {
  // Variables to track centroid, pointer to data, and useful data points.
  Eigen::Vector3f centroid{0.0, 0.0, 0.0};
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcd, "x");
  int inliers = 0, total = 0;

  // Pre-allocate matrix with enough memory.
  Eigen::MatrixXf blob_pcd(3, pcd.width * pcd.height);

  // Loop over all points to get the centroid [m].
  for (; iter_x != iter_x.end(); ++iter_x) {
    Eigen::Vector3f point{iter_x[0], iter_x[1], iter_x[2]};

    // Discard points that are all zero, or too far away.
    float distance = point.norm();

    if (distance != 0 && (std::abs(point(0)) <= params_.max_x_distance &&
                          std::abs(point(1)) <= params_.max_y_distance &&
                          std::abs(point(2)) <= params_.max_z_distance)) {
      // Add point to the Eigen point cloud.
      blob_pcd.col(inliers) = point;

      // Add information to centroid.
      centroid += point;
      inliers++;
    }
    total++;
  }
  Eigen::MatrixXf trunc_pcd;
  trunc_pcd = blob_pcd.block(0, 0, 3, inliers);
  Eigen::Vector3f final_centroid = centroid / inliers;

  // Adjust truncated distance based on centroid estimate
  params_.max_z_distance = final_centroid(2) + 0.5;

  // Downsample the point cloud if there are a lot of points (speeds up teaser)
  Eigen::MatrixXf final_pcd;
  if (trunc_pcd.cols() > params_.max_pcd_size) {
    final_pcd = BlobTracker::DownSamplePcd(trunc_pcd);
  }
  else {
    final_pcd = trunc_pcd;
  }

  // Return centroid and the Eigen point cloud.
  return std::make_pair(final_centroid, final_pcd);
}

Eigen::MatrixXf BlobTracker::DownSamplePcd(const Eigen::MatrixXf &raw_pcd) {
  std::random_device rd;
  std::mt19937 prng(rd());
  Eigen::Matrix<float, 3, Eigen::Dynamic> downsampled_pcd;
  downsampled_pcd.resize(3, params_.max_pcd_size);
  std::vector<int> pcd_idx(raw_pcd.cols());
  for (int i = 0; i < pcd_idx.size(); i++) {
    pcd_idx[i] = i;
  }
  std::shuffle(pcd_idx.begin(), pcd_idx.end(), prng);
  for (int j = 0; j < params_.max_pcd_size; j++) {
    downsampled_pcd.col(j) = raw_pcd.col(pcd_idx[j]);
  }
  return downsampled_pcd;
}

}  // namespace slam
