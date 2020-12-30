/**
 * @file SLAMNode.h
 * @brief ROS node for SLAM.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit-slam/SlamNode.h"

namespace slam {

SlamNode::SlamNode(const Params &params) : params_(params) {
  // Subscribe to the chaser IMU topic.
  imu_sub_ = nh_.subscribe(params_.imu_topic, 1,
                           &SlamNode::ImuMeasCallback, this);
  // Subscribe to the point cloud topic.
  pcd_sub_ = nh_.subscribe(params_.pcd_topic, 1,
                           &SlamNode::PointCloudCallback, this);

    // Subscribe to chaser estimated world frame state (Astrobee EKF).
  chaser_ekf_sub_ = nh_.subscribe(params_.chaser_ekf_topic, 1,
                                  &SlamNode::ChaserEkfCallback, this);

  // Subscribe to chaser ground truth pose
  chaser_gt_pose_sub_ = nh_.subscribe(params_.chaser_gt_pose_topic, 1,
                                  &SlamNode::ChaserGtPoseCallback, this);
  // Subscribe to chaser ground truth twist
  chaser_gt_twist_sub_ = nh_.subscribe(params_.chaser_gt_twist_topic, 1,
                                  &SlamNode::ChaserGtTwistCallback, this);
  // Subscribe to chaser ground truth pose
  target_gt_pose_sub_ = nh_.subscribe(params_.target_gt_pose_topic, 1,
                                  &SlamNode::TargetGtPoseCallback, this);
  // Subscribe to chaser ground truth twist
  target_gt_twist_sub_ = nh_.subscribe(params_.target_gt_twist_topic, 1,
                                  &SlamNode::TargetGtTwistCallback, this);

  // Publishers for estimates
  chaser_est_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        params_.chaser_est_pose_topic, 1);
  target_est_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        params_.target_est_pose_topic, 1);
  chaser_est_twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
        params_.chaser_est_twist_topic, 1);
  target_est_twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
        params_.target_est_twist_topic, 1);

  if (params_.publish_frontend_estimates) {
    delta_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        params_.delta_pose_topic, 1);
    // Publisher for centroid estimates.
    centroid_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
        params_.centroid_out_topic, 1);
    // Publisher for centroid's truncated point cloud.
    truncated_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        params_.target_pcd_out_topic, 1);
    // Publisher for matched cloud.
    match_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        params_.match_point_cloud_topic, 1);
    // Publisher for estimated point cloud via delta-poses.
    est_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        params_.est_point_cloud_topic, 1);
  }

  // Initialize frame index
  frame_idx_ = -1;

  // Initialize loop closure counter
  loop_closure_count_ = 0;

}

SlamNode::~SlamNode() {}

void SlamNode::ImuMeasCallback(const sensor_msgs::Imu &msg) {
  // Transform IMU accelerations to chaser body frame acceleration
  Eigen::Vector3f omega_imu;
  omega_imu << msg.angular_velocity.x,
                 msg.angular_velocity.y,
                 msg.angular_velocity.z;
  Eigen::Vector3f accel_imu;
  accel_imu << msg.linear_acceleration.x,
               msg.linear_acceleration.y,
               msg.linear_acceleration.z;

  // Rotate into chaser frame
  Eigen::Vector3f omega = params_.T_C2I.block(0, 0, 3, 3).transpose() * omega_imu;
  Eigen::Vector3f accel = params_.T_C2I.block(0, 0, 3, 3).transpose() * accel_imu;

  // IMU odometer integrates the new IMU measurement
  graph_manager_->imu_odometer_->integrateMeasurement(accel.cast<double>(),
                                                      omega.cast<double>(),
                                                      params_.imu_dt);

}

void SlamNode::PointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
  std::cout << "--- NEW POINT CLOUD CALLBACK ---" << std::endl << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  // Record time of frame for future reference
  frame_times_.push_back(ros::Time::now().toSec());
  frame_idx_++;

  // Create matrix for the downsampled, usable point cloud.
  Eigen::MatrixXf eigen_pcd;

  /// Get an estimate of the target's centroid using the BlobTracker.
  if (blob_tracker_ != nullptr) {
    Eigen::Vector3f centroid;
    std::tie(centroid, eigen_pcd) = blob_tracker_->TruncatedCentroidAndCloud(msg);
    if (params_.publish_frontend_estimates) {
        PublishCentroid(centroid, msg.header.frame_id);
    }
    // Save first centroid estimate to define the origin of the geometric frame.
    // Translation is the chaser in the geometric frame, rotation is identity.
    if (frame_idx_ == 0) {
      Eigen::Vector4f centroid_HG_h;
      Eigen::Vector4f centroid_CG_h;
      centroid_HG_h << centroid(0), centroid(1), centroid(2), 1;
      centroid_CG_h = params_.T_C2H.inverse() * centroid_HG_h;
      centroid_CG_ = centroid_CG_h.block(0, 0, 3, 1);
      T_GC0_ << 1, 0, 0, -centroid_CG_(0),
                0, 1, 0, -centroid_CG_(1),
                0, 0, 1, -centroid_CG_(2),
                0, 0, 0,        1;
    }
  }

  // Get estimate of the rigid body motion of the target with CloudOdometer and
  // the point cloud from the previous time step.
  if (cloud_odometer_ != nullptr) {
    // Convert Eigen point cloud to teaser point cloud, add to database
    teaser::PointCloud teaser_cloud;
    for (int i = 0; i < eigen_pcd.cols(); i++) {
      teaser_cloud.push_back({static_cast<float>(eigen_pcd(0,i)),
                              static_cast<float>(eigen_pcd(1,i)),
                              static_cast<float>(eigen_pcd(2,i))});
    }
    cloud_database_.push_back(teaser_cloud);

    /// Compute 3-D features, add to database
    teaser::FPFHCloudPtr features = cloud_odometer_->DetectFeatures(teaser_cloud);
    feature_database_.push_back(features);

    /// First point cloud
    if (frame_idx_ == 0) {
      // Initialize chaser pose chain
      graph_manager_->InitChaserChain(params_.T_WC0, params_.v_WC0);

      // Initialize geometric frame (target) pose chain
      graph_manager_->InitGeomChain(T_GC0_, params_.T_WC0);

      if (params_.publish_frontend_estimates) {
        PublishGtTargetTransform(target_gt_state_.pose);
        PublishGtChaserTransform(chaser_gt_state_.pose);
      }
    }
    else {
      /// Match features in previous and current frames
      std::vector<std::pair<int,int>> matches = cloud_odometer_->MatchFeatures(cloud_database_[frame_idx_ - 1],
                                                                               cloud_database_[frame_idx_],
                                                                               feature_database_[frame_idx_ - 1],
                                                                               feature_database_[frame_idx_]);
      // Randomly down-sample the matches if necessary
      std::vector<std::pair<int,int>> matches_final;
      if (matches.size() > params_.downsample_thresh) {
        matches_final = DownSampleMatches(matches);
      }
      else {
        matches_final = matches;
      }

      /// Register the matches to compute the delta pose (pose odometry) of the HazCam w/r to geometric frame
      Eigen::Matrix4f T_HiHj = cloud_odometer_->Register(cloud_database_[frame_idx_ - 1],
                                                         cloud_database_[frame_idx_],
                                                         matches_final);
      Eigen::Matrix4f T_CiCj = (params_.T_C2H.inverse() * T_HiHj * params_.T_C2H).inverse();
      std::cout << "Delta pose of point clouds (chaser w/r to geo):" << std::endl
                << T_CiCj << std::endl;

      // Check for really bad estimates, just use set it to zero if they're bad
      if (std::abs(T_CiCj(0,3)) > 0.4 || std::abs(T_CiCj(1,3)) > 0.4 || std::abs(T_CiCj(2,3)) > 0.4) {
        std::cout << "Bad estimate, setting to zero odometry." << std::endl;
        T_CiCj << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
      }


      /// Check for loop closures if there are 7 or more available frames.
      bool loop_closure_success = false;
      int loop_idx = -1;
      Eigen::Matrix4f T_CiCj_loop;

      if (frame_idx_ >= 6) {
          std::random_device rd;
          std::mt19937 prng(rd());
          std::uniform_int_distribution<int> dis(0, frame_idx_ - 2);
          // randomly pick a past frame to check for loop closure
          loop_idx = dis(prng);

          std::vector<std::pair<int,int>> matches = cloud_odometer_->MatchFeatures(cloud_database_[loop_idx],
                                                                                   cloud_database_[frame_idx_],
                                                                                   feature_database_[loop_idx],
                                                                                   feature_database_[frame_idx_]);
          // Only proceed with the loop closure if there are a lot of matches
          std::vector<std::pair<int,int>> matches_final;
          if (matches.size() > params_.loop_match_thresh) {
            matches_final = DownSampleMatches(matches);

            // Register the matches to compute the delta pose in HazCam frame (pose-odometry)
            Eigen::Matrix4f T_HiHj_loop = cloud_odometer_->Register(cloud_database_[loop_idx],
                                                                      cloud_database_[frame_idx_],
                                                                      matches_final);
            // Transform to chaser body frame
            Eigen::Matrix4f T_CiCj_loop = (params_.T_C2H.inverse() * T_HiHj_loop * params_.T_C2H).inverse();

            // TODO: fix iSAM loop closure errors.
            // For now, leave the following line commented to disable loop closures
            // graph_manager_->AddLoopClosureFactor(T_CiCj_loop, idx, frame_idx_);
            loop_closure_count_++;
            loop_closure_success = true;
            std::cout << "SUCCESSFUL LOOP CLOSURE" << std::endl;
          }
      }

      // TODO: fix iSAM loop closure errors.
      // For now, leave the following line uncommented to disable loop closures
      loop_closure_success = false;

      graph_manager_->AddFactors(T_CiCj, frame_idx_, T_CiCj_loop, loop_idx, loop_closure_success, centroid_CG_);


      /// Get current variable estimates from graph
      // TODO: figure out covariances
      // Note: I *think* covariance has rotation elements first, then translation
      // T_WCi = Chaser pose w/r to world frame (defined at target center of mass)
      // v_WCi = Chaser velocity w/r to world frame (defined at target center of mass)
      // T_GCi = Chaser pose w/r to geometric frame
      // t_GW = translation between world and geometric frame in the geometric frame (constant)
      gtsam::Symbol T('T', frame_idx_), G('G', frame_idx_), v('v', frame_idx_), t('t', 0), l('l', 0);
      Eigen::Matrix4f T_WCi = graph_manager_->estimate_.at<gtsam::Pose3>(T).matrix().cast<float>(); // Chaser pose w/r to world
      Eigen::MatrixXf T_WCi_cov = graph_manager_->isam_->marginalCovariance(T).cast<float>(); // Estimate covariance
      Eigen::Vector3f v_WCi = graph_manager_->estimate_.at<gtsam::Velocity3>(v).cast<float>(); // Chaser v w/r to world
      Eigen::Matrix3f v_WCi_cov = graph_manager_->isam_->marginalCovariance(v).cast<float>(); // Estimate covariance
      Eigen::Matrix4f T_GCi = graph_manager_->estimate_.at<gtsam::Pose3>(G).matrix().cast<float>(); // Chaser pose w/r to geometric
      Eigen::MatrixXf T_GCi_cov = graph_manager_->isam_->marginalCovariance(G).cast<float>(); // Estimate covariance
      Eigen::Vector3f t_GT = graph_manager_->estimate_.at<gtsam::Point3>(t).cast<float>(); // translation of Target frame w/r to Geometric
      Eigen::Matrix3f t_GT_cov = graph_manager_->isam_->marginalCovariance(t).cast<float>(); // Estimate covariance
      Eigen::Vector3f t_WT = graph_manager_->estimate_.at<gtsam::Point3>(l).cast<float>(); // translation of target frame w/r to world
      Eigen::Matrix3f t_WT_cov = graph_manager_->isam_->marginalCovariance(l).cast<float>(); // Estimate covariance

      // Get target pose estimate in world frame
      Eigen::Matrix4f T_WGi = T_WCi * T_GCi.inverse();
      Eigen::MatrixXf T_WGi_cov = Eigen::MatrixXf::Zero(6, 6);
      Eigen::Matrix3f R_WCi = T_WCi.block(0, 0, 3, 3);
      T_WGi_cov.block(0, 0, 3, 3) = R_WCi * T_GCi_cov.block(0, 0, 3, 3) * R_WCi.transpose() + T_WCi_cov.block(0, 0, 3, 3);
      T_WGi_cov.block(3, 3, 3, 3) = R_WCi * T_GCi_cov.block(3, 3, 3, 3) * R_WCi.transpose() + T_WCi_cov.block(3, 3, 3, 3);
      Eigen::Matrix4f T_WTi = T_WGi;
      T_WTi.block(0, 3, 3, 1) =  t_WT;

      // FOR DEBUGGING:
      Eigen::Matrix4f T_WC_truth;
      T_WC_truth << T_ISS_ekf_(0,0), T_ISS_ekf_(0,1), T_ISS_ekf_(0,2), T_ISS_ekf_(0,3) - params_.t_targ_ISS(0),
                    T_ISS_ekf_(0,1), T_ISS_ekf_(1,1), T_ISS_ekf_(1,2), T_ISS_ekf_(1,3) - params_.t_targ_ISS(1),
                    T_ISS_ekf_(0,2), T_ISS_ekf_(1,2), T_ISS_ekf_(2,2), T_ISS_ekf_(2,3) - params_.t_targ_ISS(2),
                    0, 0, 0, 1;

      std::cout << "Ground truth chaser pose in world frame " << std::endl;
      std::cout << T_WC_truth << std::endl;
      std::cout << "iSAM estimated chaser pose in world frame: " << std::endl;
      std::cout << T_WCi << std::endl;
      std::cout << "iSAM estimated chaser velocity in world frame: " << std::endl;
      std::cout << v_WCi << std::endl;
      std::cout << "iSAM estimated target pose in world frame: " << std::endl;
      std::cout << T_WTi << std::endl;

      /// Compute angular velocity
      // Get previous frame estimates (1 previous index)
      gtsam::Symbol Tprev('T', frame_idx_ - 1), Gprev('G', frame_idx_ - 1);
      Eigen::Matrix4f T_WCprev = graph_manager_->estimate_.at<gtsam::Pose3>(Tprev).matrix().cast<float>(); // Previous chaser pose wrt world
      Eigen::Matrix4f T_GCprev = graph_manager_->estimate_.at<gtsam::Pose3>(Gprev).matrix().cast<float>(); // Previous chase pose wrt world

      // Calculate angular velocities of target
      Eigen::Matrix4f T_WGprev = T_WCprev * T_GCprev.inverse(); // Previous geometric pose wrt world
      Eigen::Matrix3f R_WGi = T_WGi.block(0, 0, 3, 3); // Rotation matrix for current geometric pose wrt world
      Eigen::Matrix3f R_WGprev = T_WGprev.block(0, 0, 3, 3); // Rotation matrix for previous geometric pose wrt world
      Eigen::Vector3f omega_T_ij = ComputeAngularVelocity(R_WGi,R_WGprev, 1.0);

      // Calculate angular velocities of chaser
      Eigen::Matrix3f R_WCprev = T_WCprev.block(0, 0, 3, 3); // Rotation matrix for previous geometric pose wrt world
      Eigen::Vector3f omega_C_ij = ComputeAngularVelocity(R_WCi,R_WCprev, 1.0);
      for (int i = 0; i < 3; i++) {
        if (std::isnan(omega_C_ij(i))) {
            omega_C_ij(i) = 0.0;
        }
      }

      std::cout << "iSAM estimated target angular velocity" << std::endl;
      std::cout << omega_T_ij << std::endl;
      std::cout << "iSAM estimated chaser angular velocity" << std::endl;
      std::cout << omega_C_ij << std::endl;

      // Angular velocity covariance for both chaser and target
      Eigen::Matrix3f R_WCi_cov = T_WCi_cov.block(0, 0, 3, 3);
      Eigen::Matrix3f R_GCi_cov = T_GCi_cov.block(0, 0, 3, 3);
      Eigen::Matrix3f R_GCi = T_GCi.block(0, 0, 3, 3);
      Eigen::Matrix3f R_GCprev = T_GCprev.block(0, 0, 3, 3);

      Eigen::Matrix3f omega_cov = R_GCprev * (R_GC_prev_cov_ + R_WC_prev_cov_) * R_GCprev.transpose() +
                                  R_GCprev * R_WCprev.transpose() * R_WCi * (R_WCi_cov + R_GCi_cov) *
                                  R_WCi.transpose() * R_WCprev * R_GCprev.transpose();

      // Save covariances for next step's calculation
      R_WC_prev_cov_ = R_WCi_cov;
      R_GC_prev_cov_ = R_GCi_cov;

      /// Publish estimates
      Eigen::Vector3f v_WGi;
      v_WGi << 0.0, 0.0, 0.0;
      Eigen::Matrix3f v_WGi_cov;
      v_WGi_cov << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      PublishChaserPoseEst(T_WCi, T_WCi_cov, msg.header.frame_id);
      PublishTargetPoseEst(T_WTi, T_WGi_cov, msg.header.frame_id);
      PublishChaserTwistEst(v_WCi, omega_C_ij, v_WCi_cov, omega_cov, msg.header.frame_id);
      PublishTargetTwistEst(v_WGi, omega_T_ij, v_WGi_cov, omega_cov, msg.header.frame_id);

      if (params_.publish_frontend_estimates) {
        PublishGtTargetTransform(target_gt_state_.pose);
        PublishGtChaserTransform(chaser_gt_state_.pose);
        PublishMatchPointCloud(eigen_pcd, matches_final, msg.header.frame_id);
        PublishEstPointCloud(T_HiHj, msg.header.frame_id);
        PublishDeltaPose(T_CiCj, msg.header.frame_id);
      }
    }
    // Save point cloud for next frame
    prev_eigen_pcd_ = eigen_pcd;
  }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Callback elapsed time: " << elapsed.count() << std::endl;

    std::cout << std::endl << "--- END OF POINT CLOUD CALLBACK ---" << std::endl << std::endl << std::endl;
} // End point cloud callback

std::vector<std::pair<int,int>> SlamNode::DownSampleMatches(const std::vector<std::pair<int,int>> correspondences) {
  std::random_device rd;
  std::mt19937 prng(rd());
  std::vector<std::pair<int,int>> downsampled_matches;
  std::vector<int> match_idx(correspondences.size());
  for (int i = 0; i < match_idx.size(); i++) {
    match_idx[i] = i;
  }
  std::shuffle(match_idx.begin(), match_idx.end(), prng);
  for (int j = 0; j < params_.downsample_thresh; j++) {
    downsampled_matches.push_back(correspondences[match_idx[j]]);
  }
  return downsampled_matches;
}

void SlamNode::ChaserEkfCallback(const ff_msgs::EkfState &msg) {
  Eigen::Matrix3f rot = slam::q2dcm(msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w);
  T_ISS_ekf_ << rot(0,0), rot(0,1), rot(0,2), msg.pose.position.x,
                rot(1,0), rot(1,1), rot(1,2), msg.pose.position.y,
                rot(2,0), rot(2,1), rot(2,2), msg.pose.position.z,
                0,        0,        0,                   1;
  v_ISS_ekf_ << msg.velocity.x,
                msg.velocity.y,
                msg.velocity.z;
}

void SlamNode::ChaserGtPoseCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Matrix3f rot = slam::q2dcm(msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w);
  chaser_gt_state_.pose << rot(0,0), rot(0,1), rot(0,2), msg.pose.position.x,
                           rot(1,0), rot(1,1), rot(1,2), msg.pose.position.y,
                           rot(2,0), rot(2,1), rot(2,2), msg.pose.position.z,
                                  0,        0,        0,                   1;
}

void SlamNode::ChaserGtTwistCallback(const geometry_msgs::TwistStamped &msg) {
  chaser_gt_state_.vel << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
  chaser_gt_state_.omega << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
}

void SlamNode::TargetGtPoseCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Matrix3f rot = slam::q2dcm(msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w);
  target_gt_state_.pose << rot(0,0), rot(0,1), rot(0,2), msg.pose.position.x,
                           rot(1,0), rot(1,1), rot(1,2), msg.pose.position.y,
                           rot(2,0), rot(2,1), rot(2,2), msg.pose.position.z,
                                  0,        0,        0,                   1;
}

void SlamNode::TargetGtTwistCallback(const geometry_msgs::TwistStamped &msg) {
  target_gt_state_.vel << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
  target_gt_state_.omega << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
}

void SlamNode::SetupBlobTracker(const BlobTracker::Params &params) {
  blob_tracker_ = std::make_unique<BlobTracker>(params);
}

void SlamNode::SetupCloudOdometer(const CloudOdometer::Params &params) {
  cloud_odometer_ = std::make_unique<CloudOdometer>(params);
}

void SlamNode::SetupGraphManager(const GraphManager::Params &params) {
  graph_manager_ = std::make_unique<GraphManager>(params);
}


void SlamNode::PublishChaserPoseEst(const Eigen::Matrix4f &chaser_pose,
                                    const Eigen::MatrixXf &cov,
                                    const std::string &frame_id) {
  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.frame_id = frame_id;
  Eigen::Matrix3f R = chaser_pose.block(0, 0, 3, 3);
  Eigen::Vector4f quat = dcm2q(R);
  p.pose.pose.position.x = chaser_pose(0,3);
  p.pose.pose.position.y = chaser_pose(1,3);
  p.pose.pose.position.z = chaser_pose(2,3);
  p.pose.pose.orientation.x = quat(0);
  p.pose.pose.orientation.y = quat(1);
  p.pose.pose.orientation.z = quat(2);
  p.pose.pose.orientation.w = quat(3);
  int idx = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
        p.pose.covariance[idx] = cov(i,j);
        idx++;
    }
  }
  chaser_est_pose_pub_.publish(p);
}

void SlamNode::PublishTargetPoseEst(const Eigen::Matrix4f &target_pose,
                                    const Eigen::MatrixXf &cov,
                                    const std::string &frame_id) {
  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.frame_id = frame_id;
  Eigen::Matrix3f R = target_pose.block(0, 0, 3, 3);
  Eigen::Vector4f quat = dcm2q(R);
  p.pose.pose.position.x = target_pose(0,3);
  p.pose.pose.position.y = target_pose(1,3);
  p.pose.pose.position.z = target_pose(2,3);
  p.pose.pose.orientation.x = quat(0);
  p.pose.pose.orientation.y = quat(1);
  p.pose.pose.orientation.z = quat(2);
  p.pose.pose.orientation.w = quat(3);
  int idx = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
        p.pose.covariance[idx] = cov(i,j);
        idx++;
    }
  }
  target_est_pose_pub_.publish(p);
}

void SlamNode::PublishChaserTwistEst(const Eigen::Vector3f &chaser_vel,
                                    const Eigen::Vector3f &chaser_w,
                                    const Eigen::Matrix3f &cov_vel,
                                    const Eigen::Matrix3f &cov_w,
                                    const std::string &frame_id) {
  geometry_msgs::TwistWithCovarianceStamped p;
  p.header.frame_id = frame_id;
  p.twist.twist.linear.x = chaser_vel(0);
  p.twist.twist.linear.y = chaser_vel(1);
  p.twist.twist.linear.z = chaser_vel(2);
  p.twist.twist.angular.x = chaser_w(0);
  p.twist.twist.angular.y = chaser_w(1);
  p.twist.twist.angular.z = chaser_w(2);
  int row_cnt = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.twist.covariance[j+row_cnt*6] = cov_vel(i,j);
    }
    row_cnt++;
  }
  int next_row_cnt = 3;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.twist.covariance[j+3+next_row_cnt*6] = cov_w(i,j);
    }
    row_cnt++;
  }
  chaser_est_twist_pub_.publish(p);
}

void SlamNode::PublishTargetTwistEst(const Eigen::Vector3f &target_vel,
                                    const Eigen::Vector3f &target_w,
                                    const Eigen::Matrix3f &cov_vel,
                                    const Eigen::Matrix3f &cov_w,
                                    const std::string &frame_id) {
  geometry_msgs::TwistWithCovarianceStamped p;
  p.header.frame_id = frame_id;
  p.twist.twist.linear.x = target_vel(0);
  p.twist.twist.linear.y = target_vel(1);
  p.twist.twist.linear.z = target_vel(2);
  p.twist.twist.angular.x = target_w(0);
  p.twist.twist.angular.y = target_w(1);
  p.twist.twist.angular.z = target_w(2);
  int row_cnt = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.twist.covariance[j+row_cnt*6] = cov_vel(i,j);
    }
    row_cnt++;
  }
  int next_row_cnt = 3;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.twist.covariance[j+3+next_row_cnt*6] = cov_w(i,j);
    }
    row_cnt++;
  }
  target_est_twist_pub_.publish(p);
}

void SlamNode::PublishCentroid(const Eigen::Vector3f &centroid,
                               const std::string &frame_id) {
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id;
  p.point.x = centroid(0);
  p.point.y = centroid(1);
  p.point.z = centroid(2);

  centroid_pub_.publish(p);
}

void SlamNode::PublishPointCloud(const Eigen::MatrixXf &eigen_pcd,
                                 const std::string &frame_id) {
  sensor_msgs::PointCloud2 pcd = CreatePointCloud2(eigen_pcd);
  pcd.header.frame_id = frame_id;
  truncated_pcd_pub_.publish(pcd);
}

void SlamNode::PublishGtTargetTransform(const Eigen::Matrix4f &pose) {
  tf::Transform gt_target;
  gt_target.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = slam::dcm2q(rot);
  gt_target.setRotation(tf::Quaternion(quat(0), quat(1), quat(2), quat(3)));
  br_.sendTransform(tf::StampedTransform(gt_target, ros::Time::now(), "map", "gt_target"));
}

void SlamNode::PublishGtChaserTransform(const Eigen::Matrix4f &pose) {
  tf::Transform gt_chaser;
  gt_chaser.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = slam::dcm2q(rot);
  gt_chaser.setRotation(tf::Quaternion(quat(0), quat(1), quat(2), quat(3)));
  br_.sendTransform(tf::StampedTransform(gt_chaser, ros::Time::now(), "map", "gt_chaser"));
  tf::Transform chaser2hazcam;
  chaser2hazcam.setOrigin(tf::Vector3(0.1328, 0.0362, -0.0826));
  chaser2hazcam.setRotation(tf::Quaternion(-0.500, 0.500, -0.500, 0.500));
  br_.sendTransform(tf::StampedTransform(chaser2hazcam, ros::Time::now(), "gt_chaser", "/honey/haz_cam"));

}

void SlamNode::PublishDeltaPose(const Eigen::Matrix4f &pose,
                                const std::string &frame_id) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = pose(0,3);
  p.pose.position.y = pose(1,3);
  p.pose.position.z = pose(2,3);
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = slam::dcm2q(rot);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  p.header.frame_id = frame_id;
  delta_pose_pub_.publish(p);

}

void SlamNode::PublishMatchPointCloud(const Eigen::MatrixXf pcd,
                                      const std::vector<std::pair<int,int>> matches,
                                      const std::string &frame_id) {
  Eigen::MatrixXf match_eigen_pcd(3, matches.size());
  for (size_t i = 0; i < matches.size(); i++) {
    match_eigen_pcd.col(i) = pcd.col(matches[i].second);
  }
  sensor_msgs::PointCloud2 match_pcd = CreatePointCloud2(match_eigen_pcd);
  match_pcd.header.frame_id = frame_id;
  match_point_cloud_pub_.publish(match_pcd);
}

void SlamNode::PublishEstPointCloud(const Eigen::Matrix4f &pose_odometry,
                                    const std::string &frame_id) {
  // compute "estimated" current point cloud using the previous frame and the estimated delta-pose
  Eigen::Matrix<float, 4, Eigen::Dynamic> prev_eigen_pcd_h;
  prev_eigen_pcd_h.resize(4, prev_eigen_pcd_.cols());
  prev_eigen_pcd_h.topRows(3) = prev_eigen_pcd_;
  prev_eigen_pcd_h.bottomRows(1) = Eigen::Matrix<float, 1, Eigen::Dynamic>::Ones(prev_eigen_pcd_.cols());
  Eigen::Matrix<float, 4, Eigen::Dynamic> current_h = pose_odometry * prev_eigen_pcd_h;
  Eigen::Matrix<float, 3, Eigen::Dynamic> current = current_h.topRows(3);

  sensor_msgs::PointCloud2 est_pcd = CreatePointCloud2(current);
  est_pcd.header.frame_id = frame_id;
  est_point_cloud_pub_.publish(est_pcd);
}

}  // namespace slam
