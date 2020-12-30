/**
 * @file relative_pose_checkout.cpp
 * @brief Simple node to calculate the ground truth relative pose between chaser
 * and target spacecrafts.
 * @date May 13, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "mit-slam/GeometryUtils.h"

/// Class that computes and published the relative pose between two PoseStamped.
class RelativePoseCalculator {
 public:
  /// Constructor.
  RelativePoseCalculator(const std::string &chaser_pose_topic,
                         const std::string &target_pose_topic,
                         const std::string &relpose_publish_topic)
      : chr_topic_(chaser_pose_topic),
        tgt_topic_(target_pose_topic),
        pub_topic_(relpose_publish_topic) {}
  /// Destructor.
  ~RelativePoseCalculator() {}

  /// Initializes the publisher, subscribers, and the time synchronizer.
  void Initialize() {
    // Initialize the publisher.
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pub_topic_, 1000);

    // Initialize the subscribers.
    chaser_pose_sub_ = std::make_unique<
        message_filters::Subscriber<geometry_msgs::PoseStamped>>(
        nh_, chr_topic_, 100);

    target_pose_sub_ = std::make_unique<
        message_filters::Subscriber<geometry_msgs::PoseStamped>>(
        nh_, tgt_topic_, 100);

    // Initialize the time synchronizer using both subscribers.
    sync_ = std::make_unique<message_filters::TimeSynchronizer<
        geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>>(
        *chaser_pose_sub_, *target_pose_sub_, 1000);

    // Register the callback.
    sync_->registerCallback(&RelativePoseCalculator::RelativePoseCallback,
                            this);
  }

 private:
  /// The callback that computes the relative pose estimate from chaser to
  /// target. That is, `tfm` is such that `target` = `chaser` \oplus `tfm`.
  void RelativePoseCallback(const geometry_msgs::PoseStamped &chaser,
                            const geometry_msgs::PoseStamped &target) {
    // Get Eigen SE(3).
    Eigen::Affine3d e_chaser, e_target;
    tf::poseMsgToEigen(chaser.pose, e_chaser);
    tf::poseMsgToEigen(target.pose, e_target);

    // Compute the relative transform.
    Eigen::Affine3d relative_pose = e_chaser.inverse() * e_target;

    // Build the pose stamped for publishing.
    geometry_msgs::PoseStamped relative;
    relative.header = chaser.header;  // Copy the correct time stamp.
    // relative.header.frame_id = "chaser";  // Set the correct reference frame.

    // Set the translation.
    relative.pose.position.x = relative_pose(0, 3);
    relative.pose.position.y = relative_pose(1, 3);
    relative.pose.position.z = relative_pose(2, 3);

    // Set the orientation using a quaternion.
    Eigen::Quaterniond quat{relative_pose.rotation()};
    relative.pose.orientation.x = quat.x();
    relative.pose.orientation.y = quat.y();
    relative.pose.orientation.z = quat.z();
    relative.pose.orientation.w = quat.w();

    // Publish.
    pose_pub_.publish(relative);
  }

  /// The ROS node handle.
  ros::NodeHandle nh_;
  /// Subscriber to the pose ground truth for the chaser satellite.
  std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>>
      chaser_pose_sub_ = nullptr;
  /// Subscriber to the pose ground truth for the target satellite.
  std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>>
      target_pose_sub_ = nullptr;
  /// The time synchronizer for both pose messages.
  std::unique_ptr<message_filters::TimeSynchronizer<geometry_msgs::PoseStamped,
                                                    geometry_msgs::PoseStamped>>
      sync_ = nullptr;
  /// Publisher for the stamped relative pose.
  ros::Publisher pose_pub_;
  /// The publisher and subscribers' topics.
  std::string chr_topic_, tgt_topic_, pub_topic_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "relative_pose_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(65);

  // Create the relative pose calculator object.
  RelativePoseCalculator rpc{"/honey/loc/truth/pose", "/loc/truth/pose",
                             "/mitslam/truth/rel_pose"};
  rpc.Initialize();

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
