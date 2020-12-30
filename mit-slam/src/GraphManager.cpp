/**
 * @file GraphManager.cpp
 * @brief Class for building factors and adding to factor graph.
 * @date December 30, 2020
 * @author Tonio Teran, teran@mit.edu, Charles Oestreich (coestrei@mit.edu), Jessica Todd (jetodd@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */


#include "mit-slam/GraphManager.h"

namespace slam {

// Graph Manager constructor
GraphManager::GraphManager(const Params &params) : params_(params) {

    // Initialize preintegration
    params_.pimParams->accelerometerCovariance = params_.measured_acc_cov;
    params_.pimParams->integrationCovariance = params_.integration_error_cov;
    params_.pimParams->gyroscopeCovariance = params_.measured_omega_cov;
    params_.pimParams->biasAccCovariance = params_.bias_acc_cov;
    params_.pimParams->biasOmegaCovariance = params_.bias_omega_cov;
    params_.pimParams->biasAccOmegaInt = params_.bias_acc_omega_int;
    imu_odometer_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_.pimParams, params_.bias0);

    // Initialize Bayes tree
    isam_ = std::make_shared<gtsam::ISAM2>(params_.isam2Params);

} // end of constructor

// Graph Manager destructor
GraphManager::~GraphManager(){

    // Export any data

} // end of destructor

// Initialize first baby factor graph with priors
void GraphManager::InitChaserChain(const Eigen::Matrix4f &T_WC, const Eigen::Vector3f &v_WC) {

    // Initialise baby factor graph
    gtsam::NonlinearFactorGraph graph;

    // Initialize symbols
    gtsam::Symbol pose_sym('T', 0);
    gtsam::Symbol vel_sym('v', 0);
    gtsam::Symbol bias_sym('b', 0);
    gtsam::Symbol blob_sym('l', 0);

    // Initialize nav state using the initial astrobee estimate
    gtsam::Pose3 T0(T_WC.cast<double>());
    gtsam::Velocity3 v0(v_WC.cast<double>());
    gtsam::Point3 blob0(Eigen::Vector3d::Zero());

    // Add prior factors to graph
    gtsam::PriorFactor<gtsam::Pose3> pPrior(pose_sym, T0, params_.ppNM);
    gtsam::PriorFactor<gtsam::Velocity3> vPrior(vel_sym, v0, params_.vpNM);
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> bPrior(bias_sym, params_.bias0, params_.bpNM);
    gtsam::PriorFactor<gtsam::Point3> blobPrior(blob_sym, blob0, params_.blobNM);
    graph.push_back(pPrior);
    graph.push_back(vPrior);
    graph.push_back(bPrior);
    graph.push_back(blobPrior);

    // Add values to initial states
    gtsam::Values values;
    values.insert<gtsam::Pose3>(pose_sym, T0);
    values.insert<gtsam::Velocity3>(vel_sym, v0);
    values.insert(bias_sym, params_.bias0);
    values.insert<gtsam::Point3>(blob_sym, blob0);

    // Add baby graph to to iSAM2
    isam_->update(graph, values);

} // end of InitFactorGraph

// Initialize geometric chain
void GraphManager::InitGeomChain(const Eigen::Matrix4f &T_GC0, const Eigen::Matrix4f &T_WC) {

    // Initialize baby factor graph
    gtsam::NonlinearFactorGraph graph;

    // Initial symbols
    gtsam::Symbol geom_sym('G', 0); // NOTE: should be 0
    gtsam::Symbol t_geom_sym('t', 0); // There is only one node

    // Calculate the initial translation offset using initial pose estimate
    Eigen::Matrix3f R_WC = T_WC.block(0, 0, 3, 3);
    Eigen::Vector3f t_WC = T_WC.block(0, 3, 3, 1);
    Eigen::Vector3f t_GC0 = T_GC0.block(0, 3, 3, 1);
    // R_GC0 = Identity
    gtsam::Point3 t_GW0 = gtsam::Point3((-(R_WC.transpose() * t_WC - t_GC0)).cast<double>());

    // Generate prior factor
    gtsam::PriorFactor<gtsam::Pose3> gPrior(geom_sym, gtsam::Pose3(T_GC0.cast<double>()), params_.gpNM);
    gtsam::PriorFactor<gtsam::Point3> tPrior(t_geom_sym, t_GW0, params_.tpNM);

    // Add priors to baby factor graph
    graph.push_back(gPrior);
    graph.push_back(tPrior);

    // Add initial values
    gtsam::Values values;
    values.insert<gtsam::Pose3>(geom_sym, gtsam::Pose3(T_GC0.cast<double>()));
    values.insert<gtsam::Point3>(t_geom_sym, t_GW0);

    // Update iSAM with new baby graph and values
    isam_->update(graph, values);

} // end of InitGeomChain

// Add IMU measurements to preintegration
void GraphManager::AddImuMeasurement(const Eigen::Vector3f &accel,
                                     const Eigen::Vector3f &omega,
                                     const double dt) {

    imu_odometer_->integrateMeasurement(accel.cast<double>(), omega.cast<double>(), dt);

} // end of addImuMeasurement


// Add factors to factor graph
void GraphManager::AddFactors(const Eigen::Matrix4f &T_CiCj, const int frame_idx,
                              const Eigen::Matrix4f &T_CiCj_loop, const int loop_idx, const bool loop_closure_success,
                              const Eigen::Vector3f &centroid_CG) {

    // Get involved symbols
    // -- states
    gtsam::Symbol pose_i('T', frame_idx - 1), pose_j('T', frame_idx);
    gtsam::Symbol vel_i('v', frame_idx - 1), vel_j('v', frame_idx);
    gtsam::Symbol bias_i('b', frame_idx - 1), bias_j('b', frame_idx);
    // -- Geometric pose chain
    gtsam::Symbol geom_i('G', frame_idx - 1), geom_j('G', frame_idx);
    gtsam::Symbol t_geom('t', 0);    // Only one, same for all times.
    // Range/bearing for target COM in world frame
    gtsam::Symbol t_blob('l', 0);

    // Initialize baby graph and values
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    // Create range/bearing factor to target COM, add to graph
    double range = centroid_CG.norm() + params_.blob_range_bias;
    gtsam::Unit3 bearing(centroid_CG.cast<double>());
    graph.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
        pose_j, t_blob, bearing, range, params_.blobNM);

    // Create chaser IMU factor
    gtsam::CombinedImuFactor imuf(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, *imu_odometer_);

    // Compute chaser state initial values
    auto Ti = isam_->calculateEstimate<gtsam::Pose3>(pose_i);
    auto vi = isam_->calculateEstimate<gtsam::Velocity3>(vel_i);
    auto bi = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(bias_i);
    gtsam::NavState xi(Ti, vi);
    gtsam::NavState xj = imu_odometer_->predict(xi,bi);

    // Add new chaser IMU factor to baby factor graph
    graph.push_back(imuf);
    values.insert<gtsam::Pose3>(pose_j, xj.pose());
    values.insert<gtsam::Velocity3>(vel_j, xj.velocity());
    values.insert<gtsam::imuBias::ConstantBias>(bias_j, bi);

    // Create target odometry factor
    gtsam::BetweenFactor<gtsam::Pose3> vof(geom_i, geom_j, gtsam::Pose3(T_CiCj.cast<double>()), params_.gNM);

    // Compute initial value
    auto Gi = isam_->calculateEstimate<gtsam::Pose3>(geom_i);
    gtsam::Pose3 Gj = Gi * gtsam::Pose3(T_CiCj.cast<double>());

    // Add factors to baby factor graph
    graph.push_back(vof);
    values.insert<gtsam::Pose3>(geom_j, Gj);

    // Create a rotation kinematic factor.
    gtsam::RotationKinematicFactor rkf(pose_i, geom_i, pose_j, geom_j, t_geom, params_.rkfNM);

    // Add rotation factors to baby factor graph
    graph.push_back(rkf);

    // Update Bayes Tree
    auto start = std::chrono::high_resolution_clock::now();

    gtsam::ISAM2Result res = isam_->update(graph, values);
    for (size_t i = 0; i < params_.opt_iters; i++) isam_->update();
    estimate_ = isam_->calculateEstimate();

    // Reset IMU integration and set new bias
    imu_odometer_->resetIntegrationAndSetBias(bi);

    // Add loop closure checks if available
    if (loop_closure_success) {
        gtsam::NonlinearFactorGraph graph_loop;
        // T_CiCj_loop is relative pose between two frames i and j (j > i) where j is current frame
        gtsam::Symbol geom_a('G', loop_idx);

        // Create between factor for loop closure
        gtsam::BetweenFactor<gtsam::Pose3> lcf(geom_a, geom_j, gtsam::Pose3(T_CiCj_loop.cast<double>()), params_.lcNM);

        // Add loop closure factor to baby factor graph
        graph_loop.push_back(lcf);

        // update Bayes Tree
        isam_->update(graph_loop);
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "iSAM solver elapsed time: " << elapsed.count() << std::endl;

} // end of AddFactors

} //namespace slam
