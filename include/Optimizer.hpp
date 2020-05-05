#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

//GTSAM includes
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

//Eigen includes
#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

//Boost includes
#include <boost/circular_buffer.hpp>

//ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//STD includes
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <map>

// Local includes
#include <Parameters.hpp>
#include <DynamicsFactor.hpp>

using namespace gtsam;

using symbol_shorthand::X; //pose
using symbol_shorthand::V; //velocity
using symbol_shorthand::B; //bias
using symbol_shorthand::L; //landmark

struct Landmark{
    Landmark(uint64_t id) : id(id), isStereo_(true){}
    bool isStereo_;
    Landmark(){};
    uint64_t id; //landmark ID
    std::map<int, Eigen::Vector4d> observations; // cameraID vs measurement in 2D
    bool factorAdded;
};


class Optimizer{
    private:
        NonlinearFactorGraph graph_;
        noiseModel::Diagonal::shared_ptr pose_noise_model_;
        noiseModel::Diagonal::shared_ptr velocity_noise_model_;
        noiseModel::Diagonal::shared_ptr bias_noise_model_;
        NavState prev_state_, prop_state_;
        imuBias::ConstantBias prev_bias_;
        PreintegrationType *imu_preintegrated_;
        PreintegrationType *dynamics_preintegrated_;

        Parameters p_;

        float accel_noise_sigma = 2.0e-2;
        float gyro_noise_sigma = 1.6968e-3;
        float accel_bias_rw_sigma = 3.0e-2;
        float gyro_bias_rw_sigma = 1.9393e-3;

        int state_index_ = 0;

        bool initialized_;

        Values initial_values_;

        uint64_t previous_frame_time = 0;

        void setParams();

        std::vector<Pose3> P_IMU_CAML_;
        std::vector<Pose3> P_IMU_CAMR_;

        boost::circular_buffer<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_buffer_;
        boost::circular_buffer<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> dynamics_buffer_;
        boost::circular_buffer<std::pair<uint64_t, geometry_msgs::PoseWithCovariance>> image_buffer_;

        std::thread optimization_thread_;

        std::chrono::high_resolution_clock::time_point t1_;
        std::chrono::high_resolution_clock::time_point t2_;

    public:
        Optimizer(const Parameters& p) : p_(p){
            graph_ = gtsam::NonlinearFactorGraph();
            pose_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-2, 1e-2,1e-2,1e-2,1e-2,1e-2).finished());
            velocity_noise_model_ = noiseModel::Isotropic::Sigma(3,1e-10);
            bias_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 0.05, 0.01,0.01,0.01).finished());
            imu_buffer_.set_capacity(2000000);
            image_buffer_.set_capacity(20000);
            odom_buffer_.set_capacity(10000);
            initialized_ = false;
        };

        ~Optimizer(){};

        void startThread(){
            optimization_thread_ = std::thread(&Optimizer::optimizationLoop, this);
        };

        void initializeGraph(uint64_t);

        void addImuFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> data_to_add);

        void addDynamicsFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> dynamics_data_to_add,
                                  std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data_to_add);

        void addImageFactor(std::pair<uint64_t, geometry_msgs::PoseWithCovariance>);

        void optimizationLoop();

        void setInitialTime(uint64_t initial_time){
            previous_frame_time = initial_time;
        };

        void addImuMeasurement(const std::pair<uint64_t,Eigen::Matrix<double,7,1>>& imu_msg){
            imu_buffer_.push_back(imu_msg);
        };

        void addDynamicsMeasurement(const std::pair<uint64_t,Eigen::Matrix<double,5,1>>& dynamics_msg){
            dynamics_buffer_.push_back(dynamics_msg);
        };

        void addImageMeasurement(const std::pair<uint64_t, geometry_msgs::PoseWithCovariance>& image_msg){
            image_buffer_.push_back(image_msg);
        };

        bool isInitialized(){
            return initialized_;
        }

        int getImuBufferSize(){
            return imu_buffer_.size();
        }

        std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> getImuData(uint64_t start_time, uint64_t end_time);

        std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> getDynamicsData(uint64_t start_time, uint64_t end_time);

        boost::circular_buffer<nav_msgs::Odometry> odom_buffer_;
};



#endif
