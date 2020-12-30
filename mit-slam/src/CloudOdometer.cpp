/**
 * @file CloudOdometer.cpp
 * @brief Object for computing relative transformations between point clouds.
 * @date Nov 18, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/CloudOdometer.h"

namespace slam {

CloudOdometer::CloudOdometer(const CloudOdometer::Params &params)
    : params_(params),
      teaser_solver_(std::make_unique<teaser::RobustRegistrationSolver>(
          params.teaser_params)) {}

CloudOdometer::~CloudOdometer() {}

teaser::FPFHCloudPtr CloudOdometer::DetectFeatures(teaser::PointCloud& cloud) {
  teaser::FPFHEstimation fpfh;
  teaser::FPFHCloudPtr features = fpfh.computeFPFHFeatures(cloud,
                                                           params_.norm_radius,
                                                           params_.fpfh_radius);
  return features;
}

std::vector<std::pair<int,int>> CloudOdometer::MatchFeatures(teaser::PointCloud& src_cloud,
                                                             teaser::PointCloud& tgt_cloud,
                                                             teaser::FPFHCloudPtr& src_descriptors,
                                                             teaser::FPFHCloudPtr& tgt_descriptors) {
  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(src_cloud, tgt_cloud, *src_descriptors, *tgt_descriptors,
                                                          params_.use_absolute_scale, params_.use_crosscheck, params_.use_tuple_test, params_.tuple_scale);
  return correspondences;
}

Eigen::Matrix4f CloudOdometer::Register(teaser::PointCloud& src_cloud,
                                        teaser::PointCloud& tgt_cloud,
                                        std::vector<std::pair<int,int>> correspondences) {
  teaser_solver_->reset(params_.teaser_params);
  Eigen::Matrix4f tfm;

  teaser::RegistrationSolution result = teaser_solver_->solve(src_cloud, tgt_cloud, correspondences);

  // Parse rotation and translation solutions.
  tfm.block(0, 0, 3, 3) = result.rotation.cast<float>();
  tfm.block(0, 3, 3, 1) = result.translation.cast<float>();
  tfm(3, 3) = 1;  // Homogeneous (SE(3)).
  Eigen::Matrix4f tfm_output = fixNumerics(tfm);

  return tfm_output;
}

}  // namespace slam
