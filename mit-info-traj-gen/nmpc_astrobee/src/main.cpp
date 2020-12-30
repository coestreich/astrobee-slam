/*
  A ROS node for calling the ACADO trajectory optimization software.

  Copyright 2020 Monica Ekal, Keenan Albee

  Based on code by ALG Prasad and Sambaran Ghoshal*/

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

// C++ STL inclues
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#define NX ACADO_NX
#define NOD ACADO_NOD
#define NU ACADO_NU
#define NY ACADO_NY
#define NYN ACADO_NYN
#define N ACADO_N
#define VERBOSE 1
#define NUM_STEPS 10
#define GRANITE 1

// Global variables

std::vector<double>  Pos_des(3), current_pose(3), orientation_des(4);
double repeatingvector[17], mass_init = 9.0, ixx_init = 0.15, iyy_init = 0.15, izz_init = 0.15;
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


class updateParams
{
public:
    bool areParamsSet{false};
    double mass{mass_init};
    double ixx{ixx_init};
    double iyy{iyy_init};
    double izz{izz_init};


    void getParams(const geometry_msgs::InertiaStamped::ConstPtr& msg);
};


void updateParams::getParams(const geometry_msgs::InertiaStamped::ConstPtr& msg) {
    mass = msg->inertia.m;
    ixx = msg->inertia.ixx;
    iyy = msg->inertia.iyy;
    izz = msg->inertia.izz;
    areParamsSet = true;
}




void currentPose(const ff_msgs::EkfState::ConstPtr& msg) {
    acadoVariables.x0[0] = msg->pose.position.x;
    acadoVariables.x0[1] = msg->pose.position.y;
    acadoVariables.x0[2] = msg->pose.position.z;
    acadoVariables.x0[3] = msg->velocity.x;
    acadoVariables.x0[4] = msg->velocity.y;
    acadoVariables.x0[5] = msg->velocity.z;
    acadoVariables.x0[6] = msg->pose.orientation.x;
    acadoVariables.x0[7] = msg->pose.orientation.y;
    acadoVariables.x0[8] = msg->pose.orientation.z;
    acadoVariables.x0[9] = msg->pose.orientation.w;
    acadoVariables.x0[10] = msg->omega.x;
    acadoVariables.x0[11] = msg->omega.y;
    acadoVariables.x0[12] = msg->omega.z;
    acadoVariables.x0[13] = 0.0;
    acadoVariables.x0[14] = 0.0;
    acadoVariables.x0[15] = 0.0;
    acadoVariables.x0[16] = 0.0;
    //acadoVariables.x0[17] = 0.0;
    //acadoVariables.x0[18] = 0.0;

}



void assignreference(updateParams *ptr) {
    int i, j;

    repeatingvector[0] = ptr->mass;
    repeatingvector[1] = ptr->ixx;
    repeatingvector[2] = ptr->iyy;
    repeatingvector[3] = ptr->izz;
    repeatingvector[4] = Pos_des[0];
    repeatingvector[5] = Pos_des[1];
    repeatingvector[6] = acadoVariables.x0[2];
    repeatingvector[7] = 0.0;
    repeatingvector[8] = 0.0;
    repeatingvector[9] = 0.0;
    repeatingvector[10] = orientation_des[0];
    repeatingvector[11] = orientation_des[1];
    repeatingvector[12] = orientation_des[2];
    repeatingvector[13] = orientation_des[3];
    repeatingvector[14] = 0.0;
    repeatingvector[15] = 0.0;
    repeatingvector[16] = 0.0;

    // assign the repeating vector to acado variables od (online data)
    for (i = 0; i < N+1; i++) { // horizon
        for (j = 0; j < NOD; j++) {  //number of online data variables
            acadoVariables.od[i*NOD + j] = repeatingvector[j];
        }
    }
}

void getcostmatrix() {
    int i, j;
    for (i = 0; i < NY; i++) {
        for (j = 0; j < NY; j++) {
            if (i == j) {
                switch(j) {
                    case 0:
                        acadoVariables.W[i * NY + j] =  10; // info gain - mass
                        break;
                    case 1:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos x
                        break;
                    case 2:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos y
                        break;
                    case 3:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos z
                        break;
                    case 4:
                        acadoVariables.W[i * NY + j] = 0.1; //weight for error_vel x
                        break;
                    case 5:
                        acadoVariables.W[i * NY + j] = 0.1; //weight for error_vel y
                        break;
                    case 6:
                        acadoVariables.W[i * NY + j] = 0.1; //weight for error vel z

                    case 7:
                        acadoVariables.W[i * NY + j] = 0.001; //weight for orientation error x
                        break;
                    case 8:
                        acadoVariables.W[i * NY + j] = 0.001; //weight for orientation error y
                        break;
                    case 9:
                        acadoVariables.W[i * NY + j] = 0.001; //weight for orientation error z
                        break;
                    default:
                        acadoVariables.W[i * NY + j] = 0.01;
                }
            } else {
                acadoVariables.W[i * NY + j] = 0.0;
            }
        }
    }

    for (i = 0; i < NYN; i++) {
        for (j = 0; j < NYN; j++) {
            if (i == j) {
                acadoVariables.WN[i*NYN+j] = 10;
            } else {
                acadoVariables.WN[i * NYN + j] = 0;
            }
        }
    }
}

void assignWeightInfoGain(double time_elapsed){
    int tau = 3;//10;//3;
    int W0 = 20;
    acadoVariables.W[0] = W0*std::exp(-1.0/tau*time_elapsed);
    std::cout<<"Gain : "<<acadoVariables.W[0]<<std::endl<<std::endl;

}
void initialize_mpc() {
    int i;
    /* Initialize the states and controls. */
    for (i = 0; i <  (N + 1)*NX; ++i)  acadoVariables.x[i] = 0.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.1;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    /* Initialize the online data values */
    for (i = 0; i < NOD * (N+1); ++i) acadoVariables.od[i] = 0.0;

    /* initialize Acado NMPC with arbitrary inertial params thought to be the correct values */
    for (i = 0; i < NOD * (N+1); i = i+17) {
        acadoVariables.od[i] = mass_init;
        acadoVariables.od[i+1] = ixx_init;
        acadoVariables.od[i+2] = iyy_init;
        acadoVariables.od[i+3] = izz_init;
    }
}

void print_data() {  // Function to print the important values whenever needed.
    std::cout << "Current Position: " << acadoVariables.x0[0] << " " << acadoVariables.x0[1] << " ";
    std::cout << acadoVariables.x0[2] << " " << std::endl;
    std::cout << "Position Desired: " << acadoVariables.od[4] << " " << acadoVariables.od[5] << " ";
    std::cout << acadoVariables.od[6] << std::endl;
    std::cout << "Current orientation: " << acadoVariables.x0[6] << " " << acadoVariables.x0[7] << " ";
    std::cout << acadoVariables.x0[8] << " " << acadoVariables.x0[9] << std::endl;
    std::cout << "desired Orientation: " << acadoVariables.od[10] << " " << acadoVariables.od[11] << " ";
    std::cout << acadoVariables.od[12] << " " << acadoVariables.od[13] << std::endl;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ctl_node");
    ros::NodeHandle nh;

    updateParams params;
    updateParams* params_ptr = &params;

    ros::Subscriber sub_current_pose = nh.subscribe("gnc/ekf", 5, currentPose);
    ros::Subscriber sub_current_inertial_params = nh.subscribe("mob/inertia", 1, &updateParams::getParams, &params);

    ros::Publisher cmd = nh.advertise<ff_msgs::FamCommand>("gnc/ctl/command", 5);


    /* Some temporary variables. */
    int iter = 0;
    acado_timer t;

    /* Initialize the solver. */
    acado_initializeSolver();
    initialize_mpc();

    if ( VERBOSE ) acado_printHeader();

    /* Prepare first step */
    acado_preparationStep();
    ros::Rate rate(62.5);
    double t1, t_prev, time_elapsed;
    t1 = ros::Time::now().toSec();
    t_prev = t1;
    getcostmatrix();

    //Pos_des[0] = 10.6; Pos_des[1] = -6.5; Pos_des[2] = 4.45;
    //Pos_des[0] = 11; Pos_des[1] = -7; Pos_des[2] = 4.57;
    Pos_des[0] = 0.40; Pos_des[1] =-0.25; Pos_des[2] = 0;
    orientation_des[0] = 0.0; orientation_des[1] = 0.0; orientation_des[2] = 0.7071; orientation_des[3] = 0.7071;
    //orientation_des[0] = 0.0; orientation_des[1] = 0.0; orientation_des[2] = 0.0; orientation_des[3] = 1;

    acado_tic(&t);
    while (ros::ok()) {
        t1 = ros::Time::now().toSec();



        assignreference(params_ptr);  // this function assigns reference values to online data values.
        time_elapsed = (double)acado_toc(&t); // note down time elapsed to use in the exponential decay
        std::cout<<"time elapsed: "<<time_elapsed<<std::endl;
        assignWeightInfoGain(time_elapsed);
        std::cout<<acadoVariables.W[0]<<std::endl<<std::endl;

        ROS_INFO("%f %f %f %f",params.mass,params.ixx, params.iyy, params.izz);

        /* Perform the feedback step. */
        acado_feedbackStep();

        ff_msgs::FamCommand famcmd;
        famcmd.wrench.force.x = acadoVariables.u[0];
        famcmd.wrench.force.y = acadoVariables.u[1];
        famcmd.wrench.torque.z = acadoVariables.u[5];

        if (GRANITE == 1) {
            famcmd.wrench.force.z = 0;//acadoVariables.u[2];
            famcmd.wrench.torque.x = 0;//acadoVariables.u[3];
            famcmd.wrench.torque.y = 0;//acadoVariables.u[4];
        } else {
            famcmd.wrench.force.z = acadoVariables.u[2];
            famcmd.wrench.torque.x= acadoVariables.u[3];
            famcmd.wrench.torque.y = acadoVariables.u[4];
        }
        cmd.publish(famcmd);

        iter++;
        if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e Objective = %.3e\n\n", iter, acado_getKKT(), acado_getObjective() );

        // shift the initialization
        acado_shiftStates(2, 0, 0);
        acado_shiftControls(0);

        print_data();


        // Prepare for the next step.
        acado_preparationStep();
        ros::spinOnce();
        rate.sleep();
        std::cout << "time between cycles (secs,expected 0.016): " << t1-t_prev << std::endl;

        t_prev = t1;




    }
    /* Read the elapsed time. */
    real_t te = acado_toc(&t);

    //double t2 = ros::Time::now().toSec();
    //std::cout << "total time of simulation: " << te << std::endl;



    if ( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");
    /* Eye-candy. */

    if ( !VERBOSE )
        printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / iter);

    acado_printDifferentialVariables();
    acado_printControlVariables();

    return 0;
}
