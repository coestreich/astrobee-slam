/**
 * @file SLAMNode.h
 * @brief ROS node for SLAM.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_SLAMNODE_H_
#define MIT_SLAM_SLAMNODE_H_

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <ff_msgs/EkfState.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <cmath>

#include "mit-slam/BlobTracker.h"
#include "mit-slam/CloudOdometer.h"
#include "mit-slam/GeometryUtils.h"
#include "mit-slam/GraphManager.h"
#include "mit-slam/MsgsUtils.h"

namespace slam {

//! Ros node that encapsulates the SLAM system.
/*!
  Handles the message-triggered callbacks by instantiating a `GraphManager`
  object onto which to redirect the messages. Information fusion and inference
  is carried out by iSAM2 and GTSAM in the background.
 */
class SlamNode {
 public:
  /// Structure for bundling the `SlamNode`s parameters.
  struct Params {
    /// Desired frequency at which to operate the ROS node [hz].
    int loop_rate;
    /// Choose whether to publish the front-end estimates.
    bool publish_frontend_estimates;

    /// Default topic in which the IMU measurements will be received.
    std::string imu_topic;
    /// Default Topic in which the point clouds are being received.
    std::string pcd_topic;

    /// Default topic in which chaser ground truth pose is being received.
    std::string chaser_ekf_topic;

    /// Default topic in which chaser ground truth pose is being received.
    std::string chaser_gt_pose_topic;
    /// Default topic in which chaser ground truth twist is being received.
    std::string chaser_gt_twist_topic;

    /// Default topic in which chaser ground truth pose is being received.
    std::string target_gt_pose_topic;
    /// Default topic in which chaser ground truth twist is being received.
    std::string target_gt_twist_topic;

    /// Topic to output HazCam/Teaser pose odometry
    std::string delta_pose_topic;
    /// Topic to output the target's centroid estimates.
    std::string centroid_out_topic;
    /// Topic to output the centroid's truncated point cloud.
    std::string target_pcd_out_topic;
    /// Topic to output the match point cloud
    std::string match_point_cloud_topic;
    /// Topic to output the estimated current point cloud based on delta-pose estimate
    std::string est_point_cloud_topic;

    /// Estimate publishing topics
    std::string chaser_est_pose_topic;
    std::string chaser_est_twist_topic;
    std::string target_est_pose_topic;
    std::string target_est_twist_topic;

    /// Match down-sampling threshold & criteria for loop closure
    int downsample_thresh;
    int loop_match_thresh;

    /// IMU dt
    double imu_dt;

    /// Chaser estimated initial state w/r to world frame (world frame is centered at target COM)
    Eigen::Matrix4f T_WC0;
    Eigen::Vector3f v_WC0;

    /// IMU transformation w/r to chaser (honey) body frame (astrobee/config/robot/honey.config)
    Eigen::Matrix4f T_C2I;

    /// HazCam transformation w/r to chaser (honey) body frame (astrobee/config/robot/honey.config)
    Eigen::Matrix4f T_C2H;

    /// Target truth inertia matrix (kg*m^2)
    Eigen::Matrix3f targetJ_gt;

    /// Target ISS position (for debugging only)
    Eigen::Vector3f t_targ_ISS;

  };

  /// Internal copy of the node's parameters.
  Params params_;

  /// Default constructor.
  explicit SlamNode(const Params &params);
  /// Default destructor.
  ~SlamNode();

  /// Initializes the `BlobTracker` object using specified parameters.
  void SetupBlobTracker(const BlobTracker::Params &params);
  /// Initializes the `CloudOdometer` object using specified parameters.
  void SetupCloudOdometer(const CloudOdometer::Params &params);
  /// Initializes the `CloudOdometer` object using specified parameters.
  void SetupGraphManager(const GraphManager::Params &params);

  /// Database of feature descriptors for possible loop closures
  std::vector<teaser::FPFHCloudPtr> feature_database_;

  /// Database of point cloud frames
  std::vector<teaser::PointCloud> cloud_database_;

  /// Database of frame times
  std::vector<double> frame_times_;

  /// Current frame index
  int frame_idx_;

  /// Loop closure count
  int loop_closure_count_;

  /// Variable to hold cloud centroid w/r to chaser body frame
  Eigen::Vector3f centroid_CG_;

  /// state estimate structure
  struct State {
    Eigen::Matrix4f pose;
    Eigen::Vector3f vel;
    Eigen::Vector3f omega;
  };

  /// Chaser ISS EKF states (for debugging only)
  Eigen::Matrix4f T_ISS_ekf_;
  Eigen::Vector3f v_ISS_ekf_;

  /// Estimated world states (world frame is centered at target COM)
  State chaser_est_state_;
  State target_est_state_;

  /// Ground truth ISS states (for analysis only)
  State chaser_gt_state_;
  State target_gt_state_;

  /// Placeholders for previous estimate rotation covariance
  Eigen::Matrix3f R_WC_prev_cov_;
  Eigen::Matrix3f R_GC_prev_cov_;

  /// Geometric frame definition. Chaser body pose w/r to geometric frame, which is defined by first centroid.
  Eigen::Matrix4f T_GC0_;

  /// Utility function for down-sampling matches
  std::vector<std::pair<int,int>> DownSampleMatches(const std::vector<std::pair<int,int>> correspondences);

 private:
  /// Ros node handle.
  ros::NodeHandle nh_;

  // IMU measurements.

  /// Subscriber for the IMU information.
  ros::Subscriber imu_sub_;
  /// Callback for the IMU measurements.
  void ImuMeasCallback(const sensor_msgs::Imu &msg);

  // Point cloud measurements.

  /// Subscriber for the HazCam's point clouds.
  ros::Subscriber pcd_sub_;
  /// Callback for the point cloud measurements.
  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);

  /// Subscriber for chaser EKF in ISS frame
  ros::Subscriber chaser_ekf_sub_;
  void ChaserEkfCallback(const ff_msgs::EkfState &msg);

  /// Subscriber for chaser ground truth pose in ISS frame (evaluation analysis only)
  ros::Subscriber chaser_gt_pose_sub_;
  void ChaserGtPoseCallback(const geometry_msgs::PoseStamped &msg);
  /// Subscriber for chaser ground truth twist in ISS frame (evaluation analysis only)
  ros::Subscriber chaser_gt_twist_sub_;
  void ChaserGtTwistCallback(const geometry_msgs::TwistStamped &msg);

  /// Subscriber for target ground truth pose in ISS frame
  ros::Subscriber target_gt_pose_sub_;
  void TargetGtPoseCallback(const geometry_msgs::PoseStamped &msg);
  /// Subscriber for target ground truth twist in ISS frame
  ros::Subscriber target_gt_twist_sub_;
  void TargetGtTwistCallback(const geometry_msgs::TwistStamped &msg);


  // Internal objects for estimation tasks.

  /// Blob tracker for range and bearing measurements to target's centroid.
  std::unique_ptr<BlobTracker> blob_tracker_ = nullptr;
  /// Cloud odometer.
  std::unique_ptr<CloudOdometer> cloud_odometer_ = nullptr;
  /// Graph manager.
  std::unique_ptr<GraphManager> graph_manager_ = nullptr;


  // Outputs and publishers.

  /// Publisher for the target's centroid estimate [geometry_msgs/PointStamped].
  ros::Publisher centroid_pub_;
  /// Assembles and published a stamped point using centroid estimate.
  void PublishCentroid(const Eigen::Vector3f &centroid,
                       const std::string &frame_id);
  /// Publisher for centroid truncated point cloud [sensor_msgs/PointCloud2].
  ros::Publisher truncated_pcd_pub_;
  /// Passes on a Eigen point cloud matrix to the truncated pcd publisher.
  void PublishPointCloud(const Eigen::MatrixXf &eigen_pcd,
                         const std::string &frame_id);

  ros::Publisher delta_pose_pub_;
  void PublishDeltaPose(const Eigen::Matrix4f &pose,
                        const std::string &frame_id);

  /// Publishers for estimates
  ros::Publisher chaser_est_pose_pub_;
  void PublishChaserPoseEst(const Eigen::Matrix4f &chaser_pose,
                            const Eigen::MatrixXf &cov,
                            const std::string &frame_id);
  ros::Publisher target_est_pose_pub_;
  void PublishTargetPoseEst(const Eigen::Matrix4f &target_pose,
                            const Eigen::MatrixXf &cov,
                            const std::string &frame_id);
  ros::Publisher chaser_est_twist_pub_;
  void PublishChaserTwistEst(const Eigen::Vector3f &chaser_vel,
                             const Eigen::Vector3f &chaser_w,
                             const Eigen::Matrix3f &cov_vel,
                             const Eigen::Matrix3f &cov_w,
                             const std::string &frame_id);
  ros::Publisher target_est_twist_pub_;
  void PublishTargetTwistEst(const Eigen::Vector3f &target_vel,
                             const Eigen::Vector3f &target_w,
                             const Eigen::Matrix3f &cov_vel,
                             const Eigen::Matrix3f &cov_w,
                             const std::string &frame_id);

  /// Publisher for match point cloud [sensor_msgs/PointCloud2].
  ros::Publisher match_point_cloud_pub_;
  /// Passes on matches to the match point cloud publisher.
  void PublishMatchPointCloud(const Eigen::MatrixXf pcd,
                              const std::vector<std::pair<int,int>> matches,
                              const std::string &frame_id);
  /// Publisher for estimated point cloud via delta-poses
  ros::Publisher est_point_cloud_pub_;
  /// Previous point cloud in Eigen format needed for estimation
  Eigen::MatrixXf prev_eigen_pcd_;
  /// Passes on an odometry estimate to the estimated point cloud publisher.
  void PublishEstPointCloud(const Eigen::Matrix4f &pose_odometry,
                            const std::string &frame_id);
  /// Transform broadcaster
  tf::TransformBroadcaster br_;
  /// Passes on ground truth transform to be broadcasted to RViz
  void PublishGtTargetTransform(const Eigen::Matrix4f &pose);
  /// Passes on ground truth transform to be broadcasted to RViz
  void PublishGtChaserTransform(const Eigen::Matrix4f &pose);
};

}  // namespace slam

#endif  // MIT_SLAM_SLAMNODE_H_
