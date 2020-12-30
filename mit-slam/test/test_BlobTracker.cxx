/**
 * @file test_BlobTracker.cpp
 * @brief Unit tests for the BlobTracker class.
 * @date Apr 15, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "mit-slam/BlobTracker.h"
#include "mit-slam/GeometryUtils.h"
#include "mit-slam/MsgsUtils.h"

namespace {

class BlobTrackerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    slam::BlobTracker::Params params;
    params.max_point_distance = 100;  // [m]
    blob_tracker_ =
        std::unique_ptr<slam::BlobTracker>(new slam::BlobTracker(params));

    // Set up an Eigen point cloud inside a vector.
    eigen_pcd_xy_.emplace_back(Eigen::Vector3f{1.0, 0.0, 0.0});   // 1st point.
    eigen_pcd_xy_.emplace_back(Eigen::Vector3f{0.0, 1.0, 0.0});   // 2nd point.
    eigen_pcd_xy_.emplace_back(Eigen::Vector3f{-1.0, 0.0, 0.0});  // 3rd point.
    eigen_pcd_xy_.emplace_back(Eigen::Vector3f{0.0, -1.0, 0.0});  // 4th point.

    // Get the corresponding PointCloud2.
    pcd_xy_ = slam::CreatePointCloud2(eigen_pcd_xy_);
  }

  virtual void TearDown() {
    // TODO(tonioteran) implement me.
  }

 public:
  // Error threshold for Eigen comparisons.
  double epsilon_ = 1e-7;
  // The blob tracker to be tested.
  std::unique_ptr<slam::BlobTracker> blob_tracker_;
  // Just a sample point cloud in an Eigen vector.
  std::vector<Eigen::Vector3f> eigen_pcd_xy_;
  // Just a sample point cloud as a ROS PointCloud2 message.
  sensor_msgs::PointCloud2 pcd_xy_;
};

TEST_F(BlobTrackerTest, CentroidCalculations) {
  // Do manual centroid calculation using Eigen vectors.
  Eigen::Vector3f centroid_eigen{0.0, 0.0, 0.0};
  for (const auto &v : eigen_pcd_xy_) {
    centroid_eigen += v;
  }
  centroid_eigen = centroid_eigen / eigen_pcd_xy_.size();

  // Calculate using the blob tracker.
  Eigen::Vector3f centroid_bt = blob_tracker_->Centroid(pcd_xy_);

  // Check that they're both the same.
  ASSERT_TRUE(centroid_bt.isApprox(centroid_eigen, epsilon_));

  // ---

  // Do a couple of random tests.
  int num_points = 100;
  centroid_eigen = Eigen::Vector3f{0.0, 0.0, 0.0};
  std::vector<Eigen::Vector3f> random_eigen_pcd;
  for (int i = 0; i < num_points; ++i) {
    Eigen::Vector3f sample = slam::SampleNormalVector(3);
    centroid_eigen += sample;
    random_eigen_pcd.push_back(sample);
  }
  centroid_eigen = centroid_eigen / num_points;

  // Get the point cloud in PointCloud2 form.
  sensor_msgs::PointCloud2 random_pcd =
      slam::CreatePointCloud2(random_eigen_pcd);
  // Calculate using the blob tracker.
  centroid_bt = blob_tracker_->Centroid(random_pcd);

  // Check that they're both the same.
  ASSERT_TRUE(centroid_bt.isApprox(centroid_eigen, epsilon_));
}

}  // namespace
