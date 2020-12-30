/*
  A ROS node for calling the ACADO trajectory optimization software, with parameter updates.

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

#include <nmpc_astrobee/NMPCInstruct.h>

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
#define NU ACADO_NU  // number of inputs per timestep
#define NY ACADO_NY
#define NYN ACADO_NYN
#define N ACADO_N  // number of timesteps
#define VERBOSE 1
#define GRANITE 1 // change to receive this parameter from the config file

// Global variables
std::vector<double>  lin_des(6), orientation_des(7);  // linear and angular desireds
double online_data_vector[17], mass_init = 9.0, ixx_init = 0.15, iyy_init = 0.15, izz_init = 0.15;
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
int STATUS = 2;  // indicator of whether ACADO computations should be performed
double INFO_WEIGHT_M = 0.0;  // information weighting
double INFO_WEIGHT_IZZ = 0.0;  // information weighting

void getcostmatrix() {
    int i, j;
    for (i = 0; i < NY; i++) {
        for (j = 0; j < NY; j++) {
            if (i == j) {
                switch(j) {
                    case 0:
                        acadoVariables.W[i * NY + j] =  INFO_WEIGHT_M; // info gain - mass
                        break;
                    case 1:
                        acadoVariables.W[i * NY + j] =  INFO_WEIGHT_IZZ; // info gain - izz
                        break;
                    case 2:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos x
                        break;
                    case 3:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos y
                        break;
                    case 4:
                        acadoVariables.W[i * NY + j] = 10; //weight for error_pos z
                        break;
                    case 5:
                        acadoVariables.W[i * NY + j] = 8.0; //weight for error_vel x
                        break;
                    case 6:
                        acadoVariables.W[i * NY + j] = 8.0; //weight for error_vel y
                        break;
                    case 7:
                        acadoVariables.W[i * NY + j] = 8.0; //weight for error vel z

                    case 8:
                        acadoVariables.W[i * NY + j] = 0.05; //weight for orientation error x
                        break;
                    case 9:
                        acadoVariables.W[i * NY + j] = 0.05; //weight for orientation error y
                        break;
                    case 10:
                        acadoVariables.W[i * NY + j] = 0.05; //weight for orientation error z
                        break;
                    default:
                        acadoVariables.W[i * NY + j] = 0.2;
                }
            } else {
                acadoVariables.W[i * NY + j] = 0.0;
            }
        }
    }

    for (i = 0; i < NYN; i++) {
        for (j = 0; j < NYN; j++) {
            if (i == j) {
                acadoVariables.WN[i*NYN+j] = 50;
            } else {
                acadoVariables.WN[i * NYN + j] = 0;
            }
        }
    }
}

void setInfoOnlyCosts() {
    int i, j;
    for (i = 0; i < NY; i++) {
        for (j = 0; j < NY; j++) {
            if (i == j) {
                switch(j) {
                    case 0:
                        acadoVariables.W[i * NY + j] =  INFO_WEIGHT_M; // info gain - mass
                        break;
                    case 1:
                        acadoVariables.W[i * NY + j] =  INFO_WEIGHT_IZZ; // info gain - izz
                        break;
                    case 2:
                        acadoVariables.W[i * NY + j] = 0; //weight for error_pos x
                        break;
                    case 3:
                        acadoVariables.W[i * NY + j] = 0; //weight for error_pos y
                        break;
                    case 4:
                        acadoVariables.W[i * NY + j] = 0; //weight for error_pos z
                        break;
                    case 5:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for error_vel x
                        break;
                    case 6:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for error_vel y
                        break;
                    case 7:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for error vel z

                    case 8:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for orientation error x
                        break;
                    case 9:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for orientation error y
                        break;
                    case 10:
                        acadoVariables.W[i * NY + j] = 0.0; //weight for orientation error z
                        break;
                    default:
                        acadoVariables.W[i * NY + j] = 0.0;
                }
            } else {
                acadoVariables.W[i * NY + j] = 0.0;
            }
        }
    }

    for (i = 0; i < NYN; i++) {
        for (j = 0; j < NYN; j++) {
            if (i == j) {
                acadoVariables.WN[i*NYN+j] = 0;
            } else {
                acadoVariables.WN[i * NYN + j] = 0;
            }
        }
    }
}


/*
Subscriber callbacks
*/
class updateParams
{
public:
    double mass{mass_init};
    double ixx{ixx_init};
    double iyy{iyy_init};
    double izz{izz_init};


    void getParams(const geometry_msgs::InertiaStamped::ConstPtr& msg);
};

// The `mob/inertia` subscriber callback
void updateParams::getParams(const geometry_msgs::InertiaStamped::ConstPtr& msg) {
    ROS_INFO("Mass parameters updated!");
    mass = msg->inertia.m;
    ixx = msg->inertia.ixx;
    iyy = msg->inertia.iyy;
    izz = msg->inertia.izz;
}

// The `gnc/ctl/nmpc_instruct` subscriber callback
// Get the desired position/orientation for ACADO to track
void getDes(const nmpc_astrobee::NMPCInstruct::ConstPtr& msg) {
    ROS_INFO("Latest goal and status info updated!");
    lin_des[0] = msg->x;
    lin_des[1] = msg->y;
    lin_des[2] = msg->z;
    lin_des[3] = msg->vel_x;
    lin_des[4] = msg->vel_y;
    lin_des[5] = msg->vel_z;
    orientation_des[0] = msg->quat1;
    orientation_des[1] = msg->quat2;
    orientation_des[2] = msg->quat3;
    orientation_des[3] = msg->quat4;
    orientation_des[4] = msg->ang_vel_x;
    orientation_des[5] = msg->ang_vel_y;
    orientation_des[6] = msg->ang_vel_z;
    STATUS = msg->status;
    INFO_WEIGHT_M = msg->info_weight_m;
    INFO_WEIGHT_IZZ = msg->info_weight_Izz;

    if (STATUS == 3){
        setInfoOnlyCosts();
    }
    else if (STATUS == 1){
        getcostmatrix();
    }
}

// The `gnc/ekf` subscriber callback
// Update ACADO with latest pose information
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

/*
ACADO updating and initialization
*/

// Update the reference values and goal state for ACADO
void assignreference(updateParams *ptr) {
    int i, j;

    online_data_vector[0] = ptr->mass;
    online_data_vector[1] = ptr->ixx;
    online_data_vector[2] = ptr->iyy;
    online_data_vector[3] = ptr->izz;
    online_data_vector[4] = lin_des[0];
    online_data_vector[5] = lin_des[1];
    online_data_vector[6] = acadoVariables.x0[2];  // z
    online_data_vector[7] = lin_des[3];
    online_data_vector[8] = lin_des[4];
    online_data_vector[9] = lin_des[5];
    online_data_vector[10] = orientation_des[0];
    online_data_vector[11] = orientation_des[1];
    online_data_vector[12] = orientation_des[2];
    online_data_vector[13] = orientation_des[3];
    online_data_vector[14] = orientation_des[4];
    online_data_vector[15] = orientation_des[5];
    online_data_vector[16] = orientation_des[6];

    // assign the repeating vector to acado variables od (online data)
    for (i = 0; i < N+1; i++) { // horizon
        for (j = 0; j < NOD; j++) {  //number of online data variables
            acadoVariables.od[i*NOD + j] = online_data_vector[j];
        }
    }
}

// Assign a weighting to information gain TODO set both weights
void assignWeightInfoGain(double time_elapsed){
    int tau = 3;//10;//3;
    int W0 = 10;
    INFO_WEIGHT_M = W0*std::exp(-1.0/tau*time_elapsed);  // mass
    INFO_WEIGHT_IZZ = W0*std::exp(-1.0/tau*time_elapsed);  // MoI
}

// Set the recorded test values.
void useLatestInfoGain(){
    acadoVariables.W[0] = INFO_WEIGHT_M;
    acadoVariables.W[NY+1] = INFO_WEIGHT_IZZ;
}

void print_data() {  // Function to print the important values whenever needed.
    std::cout << "Current State: " << acadoVariables.x0[0] << " " << acadoVariables.x0[1] << " "
        << acadoVariables.x0[2] << " " << acadoVariables.x0[3] << " " << acadoVariables.x0[4] << " "
        << acadoVariables.x0[5] << " " << acadoVariables.x0[6] << " " << acadoVariables.x0[7] << " "
        << acadoVariables.x0[8] << " " << acadoVariables.x0[9] << " " << acadoVariables.x0[10] << " "
        << acadoVariables.x0[11] << " " << acadoVariables.x0[12] << " " << std::endl;

    std::cout << "Desired State: " << acadoVariables.od[4] << " " << acadoVariables.od[5] << " "
              << acadoVariables.od[6] << " " << acadoVariables.od[7] << " " << acadoVariables.od[8] << " "
              << acadoVariables.od[9] << " " << acadoVariables.od[10] << " " << acadoVariables.od[11] << " "
              << acadoVariables.od[12] << " " << acadoVariables.od[13] << " " << acadoVariables.od[14] << " "
              << acadoVariables.od[15] << " " << acadoVariables.od[16] << " " << std::endl;
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

void acado_startup() {
    // Initialize the solver.
    acado_initializeSolver();

    initialize_mpc();
    if ( VERBOSE ) acado_printHeader();
    
    // Prepare first step
    acado_preparationStep();

    // Update the costs
    getcostmatrix();
}

/*
Make a repeated call to ACADO's traj opt
*/
int main(int argc, char** argv) {
    // start the ctl_node
    ros::init(argc, argv, "nmpc_node");
    ros::NodeHandle nh;

    updateParams params;
    updateParams* params_ptr = &params;

    // Subscribers for topics of interest
    ros::Subscriber sub_current_pose = nh.subscribe("gnc/ekf", 5, currentPose);
    ros::Subscriber sub_current_inertial_params = nh.subscribe("mob/inertia", 1, &updateParams::getParams, &params);
    ros::Subscriber sub_des = nh.subscribe("gnc/ctl/nmpc_instruct", 5, getDes);

    // Publisher to send out latest plan
    ros::Publisher latest_cmd = nh.advertise<ff_msgs::FamCommand>("gnc/ctl/latest_command", N);  // received by ACADO

    // Some temporary variables.
    int iter = 0;
    acado_timer t;

    double t1, t_prev, time_elapsed;
    t1 = ros::Time::now().toSec();
    ros::Rate rate(100);  // As fast as possible!
    t_prev = t1;

    // Specify the desired setpoint as something feasible.
    orientation_des[3] = 1.0;

    acado_startup();
    acado_tic(&t);

    while (ros::ok()) {  // Publish as fast as we can!
        // Check the topic queue for callbacks  
        ros::spinOnce();
        acado_tic(&t);

        if (STATUS != 2) {  // compute and command a trajectory!
            t1 = ros::Time::now().toSec();

            /*
            ACADO-dependent calls
            */

            // Assign updated parameters and goal positions
            assignreference(params_ptr);
            useLatestInfoGain();

            time_elapsed = (double)acado_toc(&t); // note down time elapsed to use in the exponential decay
            //assignWeightInfoGain(time_elapsed);

            // Compute the NMPC feedback to reach the goal
            acado_feedbackStep();  // cannot be too different from the first call!

            // Compute the FAM forces and torques: overwrite the queue, which is size N
            for (int i = 0; i < N; ++i) {
                ff_msgs::FamCommand famcmd;
                famcmd.wrench.force.x = acadoVariables.u[0 + i*NU];
                famcmd.wrench.force.y = acadoVariables.u[1 + i*NU];
                famcmd.wrench.torque.z = acadoVariables.u[5 + i*NU];

                if (GRANITE == 1) {
                    famcmd.wrench.force.z = 0;//acadoVariables.u[2];
                    famcmd.wrench.torque.x = 0;//acadoVariables.u[3];
                    famcmd.wrench.torque.y = 0;//acadoVariables.u[4];
                } else {
                    famcmd.wrench.force.z = acadoVariables.u[2 + i*NU];
                    famcmd.wrench.torque.x= acadoVariables.u[3 + i*NU];
                    famcmd.wrench.torque.y = acadoVariables.u[4 + i*NU];
                }

                // magic numbers---which is highest speed?
                famcmd.status = 3;
                famcmd.control_mode = 2;

                // Command the FAM
                latest_cmd.publish(famcmd);
            }

            // Prepare for the next step.
            iter++;

            if( VERBOSE ) {
                std::cout << "Time betwen cycles (secs, expected 0.016): " << t1-t_prev << std::endl;
                printf("Real-Time Iteration %d:  KKT Tolerance = %.3e Objective = %.3e\n", iter, acado_getKKT(), acado_getObjective() );
                printf("Excitation: %f %f\n", INFO_WEIGHT_M, INFO_WEIGHT_IZZ);
                printf("Params: %f %f %f %f\n\n",params.mass,params.ixx, params.iyy, params.izz);
            }

             // shift the initialization
            acado_shiftStates(2, 0, 0);
            acado_shiftControls(0);

            print_data();

            acado_preparationStep();
            /*
            ACADO-dependent calls
            */

            t_prev = t1;
        }
        else if (STATUS == 2) { // do not compute, do not command
            // std::cout << "Planner is deactivated." << std::endl;
        }

        rate.sleep();
    }
    std::cout << "all done" << std::endl;

    double t2 = ros::Time::now().toSec();
    std::cout << "total time of simulation: " << t2-t1 << std::endl;

    /* Read the elapsed time. */
    real_t te = acado_toc(&t);

    if ( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");
    /* Eye-candy. */

    if ( !VERBOSE )
        printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / iter);

    //acado_printDifferentialVariables();
    //acado_printControlVariables();

    return 0;
}