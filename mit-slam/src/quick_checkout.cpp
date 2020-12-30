/**
 * @file quick_checkout.cpp
 * @brief Simple checkout test for mit-slam package.
 * @date Mar 31, 2020
 * @author tonio terán (teran@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "quick_checkout");
  ros::NodeHandle n;

  // Start a simple publisher to make sure the node is alive.
  ros::Publisher pub = n.advertise<std_msgs::String>("/mitslam/hello", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std::stringstream ss;
    ss << "Quick checkout test. Ping #" << count++;

    std_msgs::String msg;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);

    // Test the parameter server.
    int sample_int = 0;
    n.getParam("node_loop_rate", sample_int);
    double sample_double = 0;
    n.getParam("prior_pos_stddev", sample_double);
    std::string sample_string = "";
    n.getParam("imu_meas_topic", sample_string);

    ROS_INFO("Parsed loop rate: %d", sample_int);
    ROS_INFO("Parsed stddev: %f", sample_double);
    // Clear the stringstream.
    ss.str(std::string());
    ss << sample_string;
    ROS_INFO("Parsed topic: %s", ss.str().c_str());

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
