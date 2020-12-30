/**
 * @file teaser_checkout.cpp
 * @brief Simple checkout test for TEASER++ registration library.
 * @date Apr 23, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

// TODO(tonioteran) Open an issue in teaserpp's repo.
#include <iostream>

#include "mit-slam/CloudOdometer.h"

int main(int argc, char** argv) {
  std::cout << "Quick TEASER++ tests using the cloud odometer." << std::endl;

  // Configure.
  teaser::RobustRegistrationSolver::Params tparams;
  tparams.estimate_scaling = false;
  slam::CloudOdometer::Params params;
  // Set the teaserpp parameters to CloudOdometer parameters structure.
  params.teaser_params = tparams;

  // Create CloudOdometer.
  slam::CloudOdometer co{params};

  const unsigned int n = 10;
  for (unsigned int i = 0; i < n; i++) {
    // Create random point cloud, transform, and then register.
    Eigen::MatrixXd src = Eigen::MatrixXd::Random(3, 1000);
    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    src_h.resize(4, src.cols());
    src_h.topRows(3) = src;
    src_h.bottomRows(1) =
        Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(src.cols());

    // Apply an arbitrary SE(3) transformation
    Eigen::Matrix4d T;
    // clang-format off
    T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
      -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
      4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890e-01,
      0,              0,                0,              1;
    // clang-format on

    // Apply transformation
    Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

    // Register.
    Eigen::Matrix4f tfm = co.Register(src.cast<float>(), tgt.cast<float>());
    std::cout << tfm << std::endl;
  }

  return 0;
}
