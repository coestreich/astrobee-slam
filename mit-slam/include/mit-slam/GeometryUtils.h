/**
 * @file GeometryUtils.h
 * @brief Common useful functions for geometric properties.
 * @date Dec 30, 2020
 * @author tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_GEOMETRYUTILS_H_
#define MIT_SLAM_GEOMETRYUTILS_H_

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace slam {

/// Sample a normally distributed, 0-mean vector of dimention `dim` and stddev
/// of 1, encapsulated in an Eigen float vector.
Eigen::VectorXf SampleNormalVector(const int dim);

/// Create a rotation matrix from a normalized quaternion.
Eigen::Matrix3f q2dcm(const float x, const float y, const float z, const float w);

/// Create a normalized quaternion from a rotation matrix
Eigen::Vector4f dcm2q(const Eigen::Matrix3f &dcm);

/// Set very small matrix values to zero
Eigen::Matrix4f fixNumerics(const Eigen::Matrix4f &tfm_input);

/// Calculate angular velocity from two sequential pose measurements
Eigen::Vector3f ComputeAngularVelocity(const Eigen::Matrix3f &R1,
                                       const Eigen::Matrix3f &R2,
                                       const double deltaT);

/// Log/Vee map calculations for angular velocity
Eigen::Vector3f Log(const Eigen::Matrix3f &R);
Eigen::Vector3f Vee(const Eigen::Matrix3f &M);

}  // namespace slam

#endif  // MIT_SLAM_GEOMETRYUTILS_H_
