/*Copyright 2019 Monica Ekal*/

#include <ros/ros.h>
#include <ros/rate.h>



// Messages
#include <inertia_estimator/params.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/FamCommand.h>
#include <geometry_msgs/InertiaStamped.h>
#include <nmpc_astrobee/NMPCInstruct.h>

// Libs
#include <Eigen/Dense>

// C++ STL inclues
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>


Eigen::Vector3d forces, forces_body, torques, torques_;
Eigen::Vector3d accels, alpha, vels, vels_prev, omega, omega_prev;
Eigen::Quaterniond q_world, p, pRotated;

// Config values
Eigen::Vector2d com(0,0);
double mass_init = 18, izz_init = 0.25, iyy_init = 0.25, ixx_init = 0.25, dt;

ros::Time t_now, t_last;
int STATUS = 2;
int FIRST_CALL_MADE = 0;

class estimator{

    // better to use fixed arrays rather than dynamic sizing as memory is allocated on the heap for dynamic sizing and
    // numbers of rows and colums are stored as member variables
    Eigen::VectorXd y_est, x_est_EKF, forces,torques, accels, omega, omega_prev, vels, vels_prev, y;
    Eigen::MatrixXd R, K, S, I, P, H, Q, F;

public:
    estimator(): y_est(5), x_est_EKF(2), forces(2), torques(1), accels(2), omega(1), omega_prev(1), vels(2), vels_prev(2), y(5), R(5,5), K(2,5), S(5,5), I(2,2), P(2,2), H(5,2), Q(2,2),F(2,2){
        // measurement noise covariance
        R<<0.05,0,0,0,0,
                0,0.05,0,0,0,
                0, 0, 0.05, 0,0,
                0, 0, 0, 0.05,0,
                0, 0, 0, 0,   0.08;
        // convariance of the estimates
        P<<20, 0,
            0, 20;
        I<<Eigen::Matrix2d::Identity();
        // initialize parameter estimates
        x_est_EKF <<mass_init, izz_init;
        // process noise covariance
        Q<<0.0001, 0,
           0, 0.0001;
        // Jacobian of the process model with respect to estimates: x_est = x_est
        F<<1, 0,
           0, 1 ;
        vels_prev<<0,0;
    }

    void getParamsEKF(double fx, double fy, double tz,  double acc_x, double acc_y, double omega, double vel_x, double vel_y, double dt);
    void pubParams(ros::Publisher inertia_param_pub){
        inertia_estimator::params msg;
        msg.mass = x_est_EKF(0);
        msg.izz = x_est_EKF(1);
        msg.K = {K(0,0), K(0,1), K(0,2), K(0,3), K(1,4)};
        msg.P = {P(0,0), P(1,1)};
        inertia_param_pub.publish(msg);
    }

    void pubParamsMob(ros::Publisher inertia_mob_pub){
        geometry_msgs::InertiaStamped msg;
        // EKF vals
        msg.inertia.m = x_est_EKF(0);
        msg.inertia.izz = x_est_EKF(1);

        // Best guesses
        msg.inertia.ixx = 0.251700013876;
        msg.inertia.iyy = 0.251700013876;
        msg.inertia.com.x = com(0);
        msg.inertia.com.y = com(1);
        inertia_mob_pub.publish(msg);
    }

    void pubOrigParamsMob(ros::Publisher inertia_mob_pub){
        geometry_msgs::InertiaStamped msg;
        // Config vals
        msg.inertia.m = mass_init;
        msg.inertia.izz = izz_init;
        msg.inertia.ixx = ixx_init;
        msg.inertia.iyy = iyy_init;
        msg.inertia.com.x = com(0);
        msg.inertia.com.y = com(1);
        inertia_mob_pub.publish(msg);
    }

};

void getDes(const nmpc_astrobee::NMPCInstruct::ConstPtr& msg) {
    if (FIRST_CALL_MADE == 0) {
        ros::Duration(10).sleep(); // Delay between starting of fans and the robot moving - to be tuned. (onl for table-testing)
        FIRST_CALL_MADE = 1;
    }
    STATUS = msg->status;
}

void estimator::getParamsEKF(double fx, double fy, double tz, double acc_x, double acc_y, double omega_z, double vel_x, double vel_y, double dt ) {
    this->forces<<fx,fy;
    this->accels<<acc_x,acc_y;
    this->vels<<vel_x,vel_y;
    this->torques<<tz;
    //this->alpha<<alpha;
    this->omega<<omega_z;

    //std::cout<<forces*x_est_KF.inverse()*x_est_KF.inverse()<<std::endl;

    H<<-forces(0)*dt/(x_est_EKF(0)*x_est_EKF(0)),       0,
        -forces(1)*dt/(x_est_EKF(0)*x_est_EKF(0)),      0,
        -forces(0)/(x_est_EKF(0)*x_est_EKF(0)),         0,
        -forces(1)/(x_est_EKF(0)*x_est_EKF(0)),         0,
        -com(0)*accels(1) + com(1)*accels(0),         -torques(0)*dt/(x_est_EKF(1)*x_est_EKF(1));

    //std::cout<<"H"<<H<<std::endl;

    y<<vels,accels,omega;


    /* prediction equations*/
    // predict using the process equations:
    x_est_EKF = x_est_EKF;
    // predict covariance:
    P << F*P*F.transpose() + Q;
    //innovation, pre-fit
    S << H*P*H.transpose()+ R;
    //kalman gain
    K << P*H.transpose()*S.inverse();


    /* update equations */
    // A posteriori estimate of state
    y_est<<forces(0)*dt/(x_est_EKF(0)) + vels_prev(0),
            forces(1)*dt/(x_est_EKF(0)) + vels_prev(1),
            forces(0)/(x_est_EKF(0)),
            forces(1)/(x_est_EKF(0)),
            torques(0)*dt/(x_est_EKF(1))+omega_prev(0) - x_est_EKF(0)*com(0)*accels(1) + x_est_EKF(0)*com(1)*accels(0) ;



    // update the estimate
    x_est_EKF = x_est_EKF + K*(y - y_est);

    // a posteriori estimate of covariance
    P << (I - K*H)*P;

    // Display status
    std::cout<<"P :"<<P(0,0)<<" "<<0<<std::endl;
    std::cout<<"     0     "<<" "<<P(1,1)<<std::endl;
    std::cout<<"mass_est: "<<x_est_EKF(0)<<" "<<"izz_est: "<<x_est_EKF(1)<<std::endl<<std::endl;
    vels_prev = vels;
    omega_prev = omega;
    //std::cout<<"vels prev"<<vels_prev<<std::endl;
}




Eigen::VectorXd assign(double a, double b, double c) {
    Eigen::VectorXd vec(3);
    vec(0) = a;
    vec(1) = b;
    vec(2) = c;
    return vec;
}


void unpack_measurements(const ff_msgs::EkfState::ConstPtr& msg) {
    t_now = msg->header.stamp;
    vels = assign(msg->velocity.x, msg->velocity.y, msg->velocity.z );
    accels = assign(msg->accel.x, msg->accel.y, msg->accel.z);
    omega = assign(msg->omega.x, msg->omega.y, msg->omega.z);
    q_world = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                 msg->pose.orientation.y, msg->pose.orientation.z);
    q_world.normalize();
}


void unpack_forces(const ff_msgs::FamCommand::ConstPtr& msg) {
    forces_body = assign(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    torques_ = assign(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "inertia_estimator");
    ros::NodeHandle nh;
    // Subscribers for topics of interest
    ros::Subscriber sub_des = nh.subscribe("gnc/ctl/nmpc_instruct", 5, getDes);
    ros::Subscriber sub_current_pose = nh.subscribe("gnc/ekf", 1, unpack_measurements);
    ros::Subscriber sub_forces_moments = nh.subscribe("gnc/ctl/command", 1, unpack_forces);
    ros::Publisher inertia_param_pub = nh.advertise<inertia_estimator::params>("mob/estimator",1);
    ros::Publisher inertia_mob_pub = nh.advertise<geometry_msgs:: InertiaStamped>("mob/inertia",1);
    ros::Rate rate(62.5);

    double t1, t_prev = 0;
    vels_prev<<0,0,0;
    forces<<0,0,0;
    omega_prev<<0,0,0;
    estimator estim;

    ros::spinOnce();
    estim.pubOrigParamsMob(inertia_mob_pub);  // make the config file values available

    int cntr = 0;

    while (nh.ok()) {
        ros::spinOnce();
        if (STATUS != 2) {
            t1 = ros::Time::now().toSec();
            dt = t1 - t_prev;
            if (dt != 0) {
                alpha << (omega - omega_prev) / dt;
            } else {
                alpha << 0, 0, 0;
            }

            estim.getParamsEKF(forces(0), forces(1), torques(2), accels(0), accels(1), omega(2), vels(0), vels(1), dt);
            estim.pubParams(inertia_param_pub);

            // estim.pubOrigParamsMob(inertia_mob_pub);  // make the config file values available
            if (cntr % 200 == 0) {
               estim.pubParamsMob(inertia_mob_pub);
            }
            cntr += 1;


            p.w() = 0;
            p.vec() = forces_body;
            pRotated = q_world * p * q_world.inverse();
            forces = pRotated.vec();
            torques = torques_; //note torque applied for the last time step.
            t_prev = t1;
            omega_prev << omega;
            vels_prev << vels;
        }
        rate.sleep();
    }
    return 0;
}

