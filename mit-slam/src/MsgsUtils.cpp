/**
 * @file MsgsUtils.cpp
 * @brief Common useful functions for ROS messages.
 * @date Apr 15, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/MsgsUtils.h"

namespace slam {

sensor_msgs::PointCloud2 PreparePointCloud2(const unsigned int num_points) {
  sensor_msgs::PointCloud2 pcd2;

  // Start filling the fields one by one.
  pcd2.width = num_points;
  pcd2.height = 1;        // unstructured point cloud.
  pcd2.fields.resize(3);  // only x, y, and z fields.
  pcd2.fields[0].name = "x";
  pcd2.fields[1].name = "y";
  pcd2.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32.
  for (size_t d = 0; d < pcd2.fields.size(); ++d, offset += 4) {
    pcd2.fields[d].offset = offset;
    pcd2.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    pcd2.fields[d].count = 1;
  }
  pcd2.point_step = offset;  // total number of bytes after loop.
  pcd2.row_step = pcd2.point_step * pcd2.width;
  pcd2.data.resize(num_points * pcd2.point_step);  // allocate memory.
  pcd2.is_bigendian = false;
  pcd2.is_dense = false;

  return pcd2;
}

sensor_msgs::PointCloud2 CreatePointCloud2(
    const std::vector<Eigen::Vector3f> &pcd) {
  // Prepare PointCloud2 message by creating structure and preallocating memory.
  sensor_msgs::PointCloud2 pcd2 = PreparePointCloud2(pcd.size());

  // Copy the data points.
  for (size_t cp = 0; cp < pcd.size(); ++cp) {
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[0].offset],
           &pcd[cp](0), sizeof(float));
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[1].offset],
           &pcd[cp](1), sizeof(float));
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[2].offset],
           &pcd[cp](2), sizeof(float));
  }

  return pcd2;
}

sensor_msgs::PointCloud2 CreatePointCloud2(const Eigen::MatrixXf &pcd) {
  // Prepare PointCloud2 message by creating structure and preallocating memory.
  sensor_msgs::PointCloud2 pcd2 = PreparePointCloud2(pcd.cols());

  // Copy the data points.
  for (size_t cp = 0; cp < pcd.cols(); ++cp) {
    // TODO(tonioteran) This is an unnecessary extra operation. Need to find a
    // way to prevent this by taking the address of an Eigen matrix element.
    Eigen::Vector3f p = pcd.col(cp);
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[0].offset], &p(0),
           sizeof(float));
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[1].offset], &p(1),
           sizeof(float));
    memcpy(&pcd2.data[cp * pcd2.point_step + pcd2.fields[2].offset], &p(2),
           sizeof(float));
  }

  return pcd2;
}

}  // namespace slam
