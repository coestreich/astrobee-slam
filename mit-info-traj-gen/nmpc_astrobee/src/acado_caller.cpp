/*
  A ROS node for sending out control points at a defined rate.

  Copyright 2020 Monica Ekal, Keenan Albee

*/

#include <ros/ros.h>

// Action
#include <ff_msgs/DockAction.h>
//#include <Eigen/Geometry>
//#include <eigen_conversions/eigen_msg.h>

// Messages
#include <ff_msgs/EkfState.h>
#include <geometry_msgs/InertiaStamped.h>
#include <ff_msgs/FamCommand.h>

#include <acado_common.h>
#include <acado_auxiliary_functions.h>

#include <nmpc_astrobee/NMPCInstruct.h>

// Callback Queue
#include <ros/callback_queue.h>

// C++ STL inclues
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#define NX ACADO_NX
#define NOD ACADO_NOD
#define NU ACADO_NU
#define NY ACADO_NY
#define NYN ACADO_NYN
#define N ACADO_N
#define VERBOSE 1
#define GRANITE 1 // change to receive this parameter from the config file

// Global variables
int STATUS = 2;  // indicator of whether ACADO computations should be performed
ff_msgs::FamCommand famcmd = {};  // set to zero

// Get the desired position/orientation for ACADO to track
void getDes(const nmpc_astrobee::NMPCInstruct::ConstPtr& msg) {
    STATUS = msg->status;
}

// Update ACADO with latest planned command
void getLatestCmd(const ff_msgs::FamCommand latest_cmd) {
    // Compute the FAM forces and torques
    famcmd.wrench.force.x = latest_cmd.wrench.force.x;
    famcmd.wrench.force.y = latest_cmd.wrench.force.y;
    famcmd.wrench.torque.z = latest_cmd.wrench.torque.z;

    if (GRANITE == 1) {
        famcmd.wrench.force.z = 0;  // acadoVariables.u[2];
        famcmd.wrench.torque.x = 0;  // acadoVariables.u[3];
        famcmd.wrench.torque.y = 0;  // acadoVariables.u[4];
    } else {
        famcmd.wrench.force.z = latest_cmd.wrench.force.z;
        famcmd.wrench.torque.x = latest_cmd.wrench.torque.x;
        famcmd.wrench.torque.y = latest_cmd.wrench.torque.y;
    }

    // magic numbers
    famcmd.status = 3;
    famcmd.control_mode = 2;
}


/*
Make a repeated call to ACADO's traj opt
*/
int main(int argc, char** argv) {
    // start the ctl_node
    ros::init(argc, argv, "ctl_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_call_once;
    ros::CallbackQueue queue_call_once;

    // Subscribers for topics of interest
    ros::Subscriber sub_des = nh.subscribe("gnc/ctl/nmpc_instruct", 5, getDes);

    nh_call_once.setCallbackQueue(&queue_call_once);  // spinOnce will NOT call this queue's callback
    ros::Subscriber sub_latest_cmd = nh_call_once.subscribe("gnc/ctl/latest_command", N, getLatestCmd);

    // Publisher to the FAM
    ros::Publisher cmd = nh.advertise<ff_msgs::FamCommand>("gnc/ctl/command", 5);  // actually sent to FAM

    ros::Rate rate(62.5);

    double t1, t_prev;
    t1 = ros::Time::now().toSec();
    t_prev = t1;

    std::chrono::milliseconds timespan(400);
    std::this_thread::sleep_for(timespan); // Wait for a latest_command to be created

    while (ros::ok()) {
        ros::spinOnce();  // updates NMPC_instruct info
        queue_call_once.callOne(ros::WallDuration(0.0));  // updates the latest famcmd

        if (STATUS != 2) {  // compute and command a trajectory!
            t1 = ros::Time::now().toSec();
            famcmd.header.stamp = ros::Time::now();
            famcmd.header.frame_id = '/body';
            // Command the FAM with latest
            cmd.publish(famcmd);
            std::cout << famcmd.wrench.force.x << " " << famcmd.wrench.force.y << std::endl;
            std::cout << "Time betwen cycles (secs, expected 0.016): " << t1-t_prev << std::endl;
            t_prev = t1;
        }
        else if (STATUS == 2) { // do not command: fans will automatically shut off
            // std::cout << "Caller is deactivated." << std::endl;
        }

        rate.sleep();
    }
    return 0;
}