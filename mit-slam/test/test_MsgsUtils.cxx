/**
 * @file test_MsgsUtils.cxx
 * @brief Unit tests for the ROS messages utilities.
 * @date Apr 15, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include <gtest/gtest.h>

#include <vector>

#include "mit-slam/MsgsUtils.h"

namespace {

TEST(MsgsUtilsTest, EigenToPcd2) {
  // Set up an Eigen point cloud inside a vector.
  std::vector<Eigen::Vector3f> eigen_pcd_xy;
  eigen_pcd_xy.emplace_back(Eigen::Vector3f{1.0, 0.0, 0.0});   // 1st point.
  eigen_pcd_xy.emplace_back(Eigen::Vector3f{0.0, 1.0, 0.0});   // 2nd point.
  eigen_pcd_xy.emplace_back(Eigen::Vector3f{-1.0, 0.0, 0.0});  // 3rd point.
  eigen_pcd_xy.emplace_back(Eigen::Vector3f{0.0, -1.0, 0.0});  // 4th point.

  // Get the PointCloud2.
  sensor_msgs::PointCloud2 pcd = slam::CreatePointCloud2(eigen_pcd_xy);

  // Make sure that we have the same number of points.
  ASSERT_EQ(eigen_pcd_xy.size(), pcd.width);

  // Make sure the coordinates are the same.
  sensor_msgs::PointCloud2Iterator<float> x_iter(pcd, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(pcd, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(pcd, "z");
  for (unsigned int i = 0; i < eigen_pcd_xy.size();
       ++i, ++x_iter, ++y_iter, ++z_iter) {
    ASSERT_EQ(eigen_pcd_xy[i](0), x_iter[0]);
    ASSERT_EQ(eigen_pcd_xy[i](1), y_iter[0]);
    ASSERT_EQ(eigen_pcd_xy[i](2), z_iter[0]);
  }
}

}  // namespace
