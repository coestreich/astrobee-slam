/**
 * @file ParamUtils.cpp
 * @brief Common useful functions for parameter handling.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/ParamUtils.h"

namespace slam {

SlamNode::Params SlamParamsFromRos() {
  SlamNode::Params params;
  ros::param::param<int>("node_loop_rate", params.loop_rate, 62.5);  // [Hz]
  ros::param::param<bool>("publish_frontend_estimates", params.publish_frontend_estimates, true);

  /// Default ROS topic names for subscribing
  ros::param::param<std::string>("imu_topic", params.imu_topic,
                                 "/honey/hw/imu");
  ros::param::param<std::string>("pcd_topic", params.pcd_topic,
                                 "/honey/hw/depth_haz/points");
  ros::param::param<std::string>("chaser_ekf_topic", params.chaser_ekf_topic,
                                 "/honey/gnc/ekf");
  ros::param::param<std::string>("chaser_gt_pose_topic", params.chaser_gt_pose_topic,
                                 "/honey/loc/truth/pose");
  ros::param::param<std::string>("chaser_gt_twist_topic", params.chaser_gt_twist_topic,
                                 "/honey/loc/truth/twist");
  ros::param::param<std::string>("target_gt_pose_topic", params.target_gt_pose_topic,
                                 "/loc/truth/pose");
  ros::param::param<std::string>("target_gt_twist_topic", params.target_gt_twist_topic,
                                 "/loc/truth/twist");

  /// Default ROS topic names for publishing
  ros::param::param<std::string>("delta_pose_topic", params.delta_pose_topic,
                                 "/honey/mitslam/deltapose");
  ros::param::param<std::string>("centroid_out_topic", params.centroid_out_topic,
                                 "/honey/mitslam/target_centroid");
  ros::param::param<std::string>("target_pcd_out_topic", params.target_pcd_out_topic,
                                 "/honey/mitslam/target_points");
  ros::param::param<std::string>("match_point_cloud_topic", params.match_point_cloud_topic,
                                 "/honey/mitslam/match_point_cloud");
  ros::param::param<std::string>("est_point_cloud_topic", params.est_point_cloud_topic,
                                 "/honey/mitslam/est_point_cloud");
  ros::param::param<std::string>("chaser_est_pose_topic", params.chaser_est_pose_topic,
                                 "/honey/mitslam/chaser/pose");
  ros::param::param<std::string>("chaser_est_pose_topic", params.chaser_est_pose_topic,
                                 "/honey/mitslam/chaser/pose");
  ros::param::param<std::string>("chaser_est_twist_topic", params.chaser_est_twist_topic,
                                 "/honey/mitslam/chaser/twist");
  ros::param::param<std::string>("target_est_pose_topic", params.target_est_pose_topic,
                                 "/honey/mitslam/target/pose");
  ros::param::param<std::string>("target_est_twist_topic", params.target_est_twist_topic,
                                 "/honey/mitslam/target/twist");


  ros::param::param<int>("downsample_thresh", params.downsample_thresh, 200);
  ros::param::param<int>("loop_match_thresh", params.loop_match_thresh, 400);

  ros::param::param<double>("imu_dt", params.imu_dt, 0.016);  // [s]

  // Initial chaser pose estimate
  // TODO: figure out how to get a reasonable initial estimate (pretty much using truth right now)
  std::vector<double> T_WC0_ros;
  ros::param::param<std::vector<double>>("T_WC0", T_WC0_ros, {0.0, -1.0, 0.0, 0.0,
                                                              1.0, 0.0, 0.0, -2.0,
                                                              0.0, 0.0, 1.0, 0.0,
                                                              0.0, 0.0, 0.0, 1.0});
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
        params.T_WC0(i,j) = T_WC0_ros[k];
        k++;
    }
  }

  // Initial chaser velocity estimate
  // TODO: figure out how to get a reasonable initial estimate (pretty much using truth right now)
  std::vector<double> v_WC0_ros;
  ros::param::param<std::vector<double>>("v_WC0", v_WC0_ros, {0.0, 0.0, 0.0});
  params.v_WC0 << v_WC0_ros[0], v_WC0_ros[1], v_WC0_ros[2];

  /// Non-changing parameters (no ROS param setting)
  // Chaser to IMU transform
  params.T_C2I << 0, -1, 0, 0.0247,
                  1,  0, 0, 0.0183,
                  0,  0, 1, 0.0094,
                  0,  0, 0,      1;

  // Chaser to HazCam transform
  params.T_C2H << 0, -1,  0,  0.1328,
                  0,  0, -1,  0.0362,
                  1,  0,  0, -0.0826,
                  0,  0,  0,       1;

  // Target truth inertia tensor
  params.targetJ_gt << 0.153,   0.0, 0.0,
                         0.0, 0.143, 0.0,
                         0.0,   0.0, 0.162;

  // Target truth position
  params.t_targ_ISS << 10.9, -6.65, 4.9;


  return params;
}

BlobTracker::Params BlobParamsFromRos() {
  BlobTracker::Params params;
  ros::param::param<float>("max_x_distance", params.max_x_distance, 0.5);  // [m]
  ros::param::param<float>("max_y_distance", params.max_y_distance, 0.5);  // [m]
  ros::param::param<float>("max_z_distance", params.max_z_distance, 2.5);  // [m]
  ros::param::param<int>("max_pcd_size", params.max_pcd_size, 2000); // [points]

  return params;
}

CloudOdometer::Params CloudParamsFromRos() {
  CloudOdometer::Params params;

  // Teaser parameters.
  teaser::RobustRegistrationSolver::Params tparams;
  // Square of ratio between acceptable noise and noise bound. Usually set to 1.
  ros::param::param<double>("teaser_cbar2", tparams.cbar2, 1.0);
  // Registration is much faster when not estimating scale.
  ros::param::param<bool>("teaser_estimate_scaling", tparams.estimate_scaling, false);
  // A bound on the noise of each provided measurement.
  ros::param::param<double>("teaser_noise_bound", tparams.noise_bound, 0.008);
  // Factor to multiple/divide the control parameter in the GNC algorithm.
  ros::param::param<double>("teaser_rotation_gnc_factor",
                            tparams.rotation_gnc_factor, 1.4);
  // Cost threshold for the GNC rotation estimators.
  ros::param::param<double>("teaser_rotation_cost_threshold",
                            tparams.rotation_cost_threshold, 0.00001);
  // The algorithm to be used for estimating rotation.
  tparams.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  // Maximum iterations allowed for the GNC rotation estimator
  tparams.rotation_max_iterations = 100;
  // Set the teaserpp parameters to CloudOdometer parameters structure.
  params.teaser_params = tparams;

  // Feature detection thresholds
  // For 2 meter scenario:
  ros::param::param<double>("norm_radius", params.norm_radius, 0.0375);
  ros::param::param<double>("fpfh_radius", params.fpfh_radius, 0.045);

  // Matching parameters
  ros::param::param<bool>("use_absolute_scale", params.use_absolute_scale, true);
  ros::param::param<bool>("use_crosscheck", params.use_crosscheck, true);
  ros::param::param<bool>("use_tuple_test", params.use_tuple_test, false);
  ros::param::param<double>("tuple_scale", params.tuple_scale, 0.975);

  return params;
}


GraphManager::Params GraphParamsFromRos() {
  GraphManager::Params params;

  int opt_iters_ros;
  ros::param::param<int>("opt_iters", opt_iters_ros, 20);
  params.opt_iters = opt_iters_ros;

  // TODO: better ways to do this for Astrobee cube (SPHERES was spherical)
  ros::param::param<double>("blob_range_bias", params.blob_range_bias, 0.07);

  /// Noise models for factor graph
  // TODO: Tune these for better accuracy

  // Chaser prior noise
  // Note: GTSAM convention is rotation noise on the first 3 components, then translation
  // Bias order is accelBias, gyroBias, as in imuBias::ConstantBias.vector()
  std::vector<double> ppNM_ros;
  std::vector<double> vpNM_ros;
  std::vector<double> bpNM_ros;
  ros::param::param<std::vector<double>>("ppNM", ppNM_ros, {0.01, 0.01, 0.01, 0.001, 0.001, 0.001});
  ros::param::param<std::vector<double>>("vpNM", vpNM_ros, {0.005, 0.005, 0.005});
  ros::param::param<std::vector<double>>("bpNM", bpNM_ros, {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
  auto ppNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << ppNM_ros[0], ppNM_ros[1], ppNM_ros[2], ppNM_ros[3], ppNM_ros[4], ppNM_ros[5]).finished());
  auto vpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << vpNM_ros[0], vpNM_ros[1], vpNM_ros[2]).finished());
  auto bpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << bpNM_ros[0], bpNM_ros[1], bpNM_ros[2], bpNM_ros[3], bpNM_ros[4], bpNM_ros[5]).finished());

  // Target prior noise
  // NOTE: from Tonio: this should actually be equality, so we place a prior with
  // extremely low uncertainty. Could try to change to a PoseEquality factor
  auto gpNM = gtsam::noiseModel::Isotropic::Sigma(6, 1e-10); // [m, rad]
  // Noise model for prior on initial translation offset from geometric frame
  double tpNM_mag;
  ros::param::param<double>("tpNM", tpNM_mag, 3);
  auto tpNM = gtsam::noiseModel::Isotropic::Sigma(3, tpNM_mag); // [m]

  // LIDAR odometry noise
  // Not sure if this is good for loop closures, but will keep them the same for now.
  std::vector<double> gNM_ros;
  std::vector<double> lcNM_ros;
  ros::param::param<std::vector<double>>("gNM", gNM_ros, {0.02, 0.02, 0.02, 0.04, 0.04, 0.04});
  ros::param::param<std::vector<double>>("lcNM", lcNM_ros, {0.02, 0.02, 0.02, 0.04, 0.04, 0.04});
  auto gNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gNM_ros[0], gNM_ros[1], gNM_ros[2], gNM_ros[3], gNM_ros[4], gNM_ros[5]).finished());
  auto lcNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << lcNM_ros[0], lcNM_ros[1], lcNM_ros[2], lcNM_ros[3], lcNM_ros[4], lcNM_ros[5]).finished());

  // Rotation kinematic factor noise
  double rkfNM_mag;
  ros::param::param<double>("rkfNM", rkfNM_mag, 0.05);
  auto rkfNM = gtsam::noiseModel::Isotropic::Sigma(3, rkfNM_mag);

  // range/bearing model for center of mass estimates
  std::vector<double> blobNM_ros;
  ros::param::param<std::vector<double>>("blobNM", blobNM_ros, {0.05, 0.05, 0.05});
  auto blobNM = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << blobNM_ros[0], blobNM_ros[1], blobNM_ros[2]).finished());

  params.ppNM = ppNM;
  params.vpNM = vpNM;
  params.bpNM = bpNM;
  params.gpNM = gpNM;
  params.tpNM = tpNM;
  params.gNM = gNM;
  params.lcNM = lcNM;
  params.rkfNM = rkfNM;
  params.blobNM = blobNM;

  // IMU parameters (Epson G362)
  // Also referenced https://github.com/haidai/gtsam/blob/master/examples/ImuFactorsExample.cpp
  double accel_noise_sigma;
  double gyro_noise_sigma;
  double accel_bias_rw_sigma;
  double gyro_bias_rw_sigma;
  double integration_error_cov_factor;
  double bias_acc_omega_int_factor;
  ros::param::param<double>("accel_noise_sigma", accel_noise_sigma, 0.00021929);
  ros::param::param<double>("gyro_noise_sigma", gyro_noise_sigma, 0.00001655);
  ros::param::param<double>("accel_bias_rw_sigma", accel_bias_rw_sigma, 0.005060);   // assuming 62.5 Hz operation
  ros::param::param<double>("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 0.0000055192);   // assuming 62.5 Hz operation
  ros::param::param<double>("integration_error_cov_factor", integration_error_cov_factor, 1e-8);
  ros::param::param<double>("bias_acc_omega_int_factor", bias_acc_omega_int_factor, 1e-5);

  gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma, 2);
  gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma, 2);
  gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*integration_error_cov_factor;
  gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma, 2);
  gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma, 2);
  gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6)*bias_acc_omega_int_factor;

  params.accel_noise_sigma = accel_noise_sigma;
  params.gyro_noise_sigma = gyro_noise_sigma;
  params.accel_bias_rw_sigma = accel_bias_rw_sigma;
  params.gyro_bias_rw_sigma = gyro_bias_rw_sigma;
  params.measured_acc_cov = measured_acc_cov;
  params.measured_omega_cov = measured_omega_cov;
  params.integration_error_cov = integration_error_cov;
  params.bias_acc_cov = bias_acc_cov;
  params.bias_omega_cov = bias_omega_cov;
  params.bias_acc_omega_int = bias_acc_omega_int;

  return params;
}


}  // namespace slam
