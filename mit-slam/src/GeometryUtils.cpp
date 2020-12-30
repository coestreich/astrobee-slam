/**
 * @file GeometryUtils.cpp
 * @brief Common useful functions for geometric properties.
 * @date Dec 30, 2020
 * @author tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/GeometryUtils.h"

namespace slam {

Eigen::VectorXf SampleNormalVector(const int dim) {
  // NOLINTNEXTLINE
  static std::mt19937 gen{std::random_device{}()};
  static std::normal_distribution<float> dist;
  return Eigen::VectorXf{dim}.unaryExpr([](float x) { return dist(gen); });
}

Eigen::Matrix3f q2dcm(const float x, const float y, const float z, const float w) {
  Eigen::Matrix3f rot;
  // clang-format off
  rot << 1 - 2*pow(y, 2) - 2*pow(z, 2), 2*x*y - 2*z*w, 2*x*z + 2*y*w,
         2*x*y + 2*z*w, 1 - 2*pow(x, 2) - 2*pow(z, 2), 2*y*z - 2*x*w,
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*pow(x, 2) - 2*pow(y, 2);
  // clang-format on
  return rot;

}

Eigen::Vector4f dcm2q(const Eigen::Matrix3f &dcm) {

    Eigen::Vector4f q;
    double tr = dcm(0,0) + dcm(1,1) + dcm(2,2);
    if(tr > 0)
    {
      double S = sqrt(tr + 1) * 2;
      q(3) = 0.25 * S;
      q(0) = (dcm(2,1) - dcm(1,2)) / S;
      q(1) = (dcm(0,2) - dcm(2,0)) / S;
      q(2) = (dcm(1,0) - dcm(0,1)) / S;

    } else if(dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2))
    {
      double S = sqrt(1.0 + dcm(0,0) - dcm(1,1) - dcm(2,2)) * 2;
      q(3) = (dcm(2,1) - dcm(1,2)) / S;
      q(0) = 0.25 * S;
      q(1) = (dcm(0,1) + dcm(1,0)) / S;
      q(2) = (dcm(0,2) + dcm(2,0)) / S;

    } else if(dcm(1,1) > dcm(2,2))
    {
      double S = sqrt(1.0 + dcm(1,1) - dcm(0,0) - dcm(2,2)) * 2;
      q(3) = (dcm(0,2) - dcm(2,0)) / S;
      q(0) = (dcm(0,1) + dcm(1,0)) / S;
      q(1) = 0.25 * S;
      q(2) = (dcm(1,2) + dcm(2,1)) / S;

    } else
    {
      double S = sqrt(1.0 + dcm(2,2) - dcm(0,0) - dcm(1,1)) * 2;
      q(3) = (dcm(1,0) - dcm(0,1)) / S;
      q(0) = (dcm(0,2) + dcm(2,0)) / S;
      q(1) = (dcm(1,2) + dcm(2,1)) / S;
      q(2) = 0.25 * S;
    }
    return q;

}

// eliminate NaNs, enforce last row of T matrix to be [0 0 0 1]
Eigen::Matrix4f fixNumerics(const Eigen::Matrix4f &tfm_input) {
  Eigen::Matrix4f tfm_output = tfm_input;
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
        if (std::isnan(tfm_output(i,j)) || ((i == 3) && (j == 0 || j == 1 || j == 2))) {
            tfm_output(i,j) = 0.0;
        }
    }
  }
  return tfm_output;
}

// Calculate angular velocity
Eigen::Vector3f ComputeAngularVelocity(const Eigen::Matrix3f &R1,
                                       const Eigen::Matrix3f &R2,
                                       const double deltaT) {
  Eigen::Matrix3f deltaR = R1.transpose() * R2;
  Eigen::Vector3f deltaTheta = Log(deltaR);
  return (1.0/deltaT)*deltaTheta;

} // end of ComputeAngularVelocity

// Log function
Eigen::Vector3f Log(const Eigen::Matrix3f &R) {

   if (R.isApprox(Eigen::Matrix3f::Identity()))
    return Eigen::Vector3f::Zero();

   double mag = std::acos((R.trace() - 1.0) /2.0);
   Eigen::Vector3f theta = (mag / (2 * std::sin(mag))) * Vee(R - R.transpose());
   return theta;

} // end of Log function

// Vector function
Eigen::Vector3f Vee(const Eigen::Matrix3f &M){

   Eigen::Vector3f vec{-M(1,2), M(0,2), -M(0,1)};
   return vec;

} // end of Vector function

}  // namespace slam

