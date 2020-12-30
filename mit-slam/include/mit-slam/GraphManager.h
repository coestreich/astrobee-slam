/**
 * @file GraphManager.h
 * @brief Class for building factors and adding to factor graph.
 * @date December 30, 2020
 * @author Tonio Teran, teran@mit.edu, Charles Oestreich (coestrei@mit.edu), Jessica Todd (jetodd@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

 #ifndef MIT_SLAM_GRAPHMANAGER_H_
 #define MIT_SLAM_GRAPHMANAGER_H_

 // Standard Includes
 #include <gtsam/inference/Symbol.h>
 #include <gtsam/navigation/CombinedImuFactor.h>
 #include <gtsam/nonlinear/ISAM2.h>
 #include <gtsam/nonlinear/NonlinearFactorGraph.h>
 #include <gtsam/slam/BetweenFactor.h>
 #include <gtsam/slam/PriorFactor.h>
 #include <gtsam/slam/BearingRangeFactor.h>
 #include <Eigen/Dense>
 #include <fstream>
 #include <memory>
 #include <queue>
 #include <random>
 #include <string>
 #include <unordered_map>
 #include <vector>

 // Custom Includes
 #include "mit-slam/RotationKinematicFactor.h"

 namespace slam {


 /**
  * @class GraphManager
  * @brief Handle sensor messages to run GTSAM-based backend inference
  *
  * Keeps track of entire problem's factor graph and Bayes tree. States
  * include the inspector's navigation state and IMU bias (symbols T, v
  * and b respectively). Lidar odometry pose graph involves inspector poses
  * wrt geometric frame (symbols G and t). An additional variable is included to estimate
  * the location of the target's centre of mass (symbol l);
  */
  class GraphManager {
    public:
        /// Structure for bundling the 'GraphManager's parameters
        struct Params {
          gtsam::imuBias::ConstantBias bias0{gtsam::Vector6::Zero()}; // Initial IMU bias

          // TODO Add various noise models as parameters
          gtsam::noiseModel::Diagonal::shared_ptr ppNM;
          gtsam::noiseModel::Diagonal::shared_ptr vpNM;
          gtsam::noiseModel::Diagonal::shared_ptr bpNM;
          gtsam::noiseModel::Isotropic::shared_ptr gpNM;
          gtsam::noiseModel::Isotropic::shared_ptr tpNM;
          gtsam::noiseModel::Isotropic::shared_ptr rkfNM;
          gtsam::noiseModel::Diagonal::shared_ptr gNM;
          gtsam::noiseModel::Diagonal::shared_ptr lcNM;
          gtsam::noiseModel::Diagonal::shared_ptr blobNM;

          double accel_noise_sigma;
          double gyro_noise_sigma;
          double accel_bias_rw_sigma;
          double gyro_bias_rw_sigma;
          gtsam::Matrix33 measured_acc_cov;
          gtsam::Matrix33 measured_omega_cov;
          gtsam::Matrix33 integration_error_cov;
          gtsam::Matrix33 bias_acc_cov;
          gtsam::Matrix33 bias_omega_cov;
          gtsam::Matrix66 bias_acc_omega_int;


          // Combined IMU factors (GTSAM IMU Preintegration parameters) (do we need these?)
          boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> pimParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

          // Inference Parameters
          gtsam::ISAM2Params isam2Params; // Bayes tree parameters for iSAM
          size_t opt_iters;           // Gauss-Newton iterations per step

          // blob range bias (since point cloud centroids aren't at Astrobee's COM)
          double blob_range_bias;

        };

        /// Internal copy of the node's parameters.
        Params params_;

        /**
         * @brief Default constructor
         */
         explicit GraphManager(const Params &params);

        /**
         * @brief Default destructor
         */
         ~GraphManager();

         /**
          * @brief Initialize the estimator using the lidar frame's ID
          * @param frame - timestamped lidar frame
          */

         void InitChaserChain(const Eigen::Matrix4f &T_WC, const Eigen::Vector3f &v_WC);

         /**
          * @brief Initialize geometric pose chain from lidar odometry data
          * @param[in] T_GC0 - Initial transform between geometric and body frame (T_B0B0)
          * @param[in] T_WC0 - Initial inspector pose wrt inertial frame (T_WB0)
          */
         void InitGeomChain(const Eigen::Matrix4f &T_GC0, const Eigen::Matrix4f &T_WC);

         /**
          * @brief Preintegrates IMU measurement
          * @param accel - Vector with accelerometer measurements
          * @param omega - Vector with gyroscope measurements
          * @param dt    - Double with period between IMU measurements
          */
         void AddImuMeasurement(const Eigen::Vector3f &accel,
                                const Eigen::Vector3f &gyro,
                                const double dt);

         /**
          * @brief Add IMU, pose odometry, and rotation kinematic factors
          * @param[in] T_CiCj - Odometric relative pose measurement, body frame
          * @param[in] frame_idx_ - Current frame index information
          *
          * Inertial information contained within member variable imu_odometer_
          */
         void AddFactors(const Eigen::Matrix4f &T_CiCj, const int frame_idx,
                              const Eigen::Matrix4f &T_CiCj_loop, const int loop_idx, const bool loop_closure_success,
                              const Eigen::Vector3f &centroid_CG);

         // Objects
         std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_odometer_ = nullptr; // Object for IMU preintegration between keyframe
         std::shared_ptr<gtsam::ISAM2> isam_ = nullptr; // Bayes tree for iSAM

         // Latest estimate of all factor graph variables
         gtsam::Values estimate_;




  } ; // class GraphManager




 } // namespace slam

 #endif  // MIT_SLAM_GRAPHMANAGER_H_
