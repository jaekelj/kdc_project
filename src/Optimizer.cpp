/*
*  Some sections are taken from GTSAM example: https://github.com/haidai/gtsam/blob/master/examples/ImuFactorsExample.cpp
*/

#include "Optimizer.hpp"

void Optimizer::initializeGraph(uint64_t current_time){


    //Format for initial state (NED) is: posx, posy, posz, qx, qy, qz, qw, velx, vely, velz

    // setParams(std::string file);

    GaussNewtonParams parameters;
    parameters.setVerbosity("ERROR");

    Vector3 init_a(0,0,0);

    for (int i = 0; i < imu_buffer_.size(); i ++){
        init_a[0] += imu_buffer_[i].second[1];
        init_a[1] += imu_buffer_[i].second[2];
        init_a[2] += imu_buffer_[i].second[3];

    }
    std::cout << "buffer size is " << imu_buffer_.size() << std::endl;
    init_a /= imu_buffer_.size();

    Eigen::Vector3d e_acc = init_a.normalized();
    Eigen::Vector3d ez_W(0.0, 0.0, 1.0);
    Eigen::Matrix<double, 6, 1> poseIncrement;
    poseIncrement.tail<3>() = Eigen::Vector3d::Zero();
    poseIncrement.head<3>() = ez_W.cross(e_acc).normalized();
    double angle = std::acos(ez_W.transpose() * e_acc);
    poseIncrement.head<3>() *= angle;
    Pose3 prior_pose;
    prior_pose = prior_pose*Pose3::Expmap(-poseIncrement);

    std::cout << "Initial pose is " << prior_pose << std::endl;


    Vector3 prior_velocity(0,0,0);
    imuBias::ConstantBias prior_bias; //Assume zeros bias
    // Values initial_values_;

    initial_values_.insert(X(0), prior_pose);
    initial_values_.insert(V(0), prior_velocity);
    initial_values_.insert(B(0), prior_bias);

    graph_.add(PriorFactor<Pose3>(X(state_index_), prior_pose, pose_noise_model_));
    graph_.add(PriorFactor<Vector3>(V(state_index_), prior_velocity, velocity_noise_model_));
    graph_.add(PriorFactor<imuBias::ConstantBias>(B(state_index_), prior_bias, bias_noise_model_));

    Matrix33 cov_a = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 cov_g = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 cov_int = Matrix33::Identity(3,3)*1e-3; // error committed in integrating position from velocities
    Matrix33 cov_ba = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 cov_bg = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 cov_int_bag = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    //These params are set from IMU calibration
    p->accelerometerCovariance = cov_a; 
    p->integrationCovariance = cov_int; 
    p->gyroscopeCovariance = cov_g; 
    p->biasAccCovariance = cov_ba; 
    p->biasOmegaCovariance = cov_bg; 
    p->biasAccOmegaInt = cov_int_bag;

    imu_preintegrated_ = new PreintegratedCombinedMeasurements(p,prior_bias);

    prev_state_ = NavState(prior_pose, prior_velocity);
    prev_bias_ = prior_bias;
    prop_state_ = prev_state_;

    initialized_ = true;

    t1_ =  std::chrono::high_resolution_clock::now(); 
    t2_ =  std::chrono::high_resolution_clock::now(); 

    // GaussNewtonOptimizer optimizer(graph_, initials, parameters);
}

void Optimizer::addImuFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> data_to_add)
{
    std::cout << "Adding " << data_to_add.size() << " imu measurents in this factor" << std::endl;
    NonlinearFactorGraph imuFactors_;
    std::vector<int> imuFactorTypes_;
 
    for (std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>>::iterator it = data_to_add.begin() ; it != data_to_add.end(); ++it)
    {
        Eigen::Matrix<double,7,1> imuMeasurement = it->second; 
        imu_preintegrated_->integrateMeasurement(Vector3(imuMeasurement[1],imuMeasurement[2],imuMeasurement[3]),
                                           Vector3(imuMeasurement[4],imuMeasurement[5],imuMeasurement[6]), 
                                           imuMeasurement[0]);
    }


    PreintegratedCombinedMeasurements *preint_imu = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
    
    CombinedImuFactor imu_factor(X(state_index_-1), V(state_index_-1),
                 X(state_index_  ), V(state_index_  ),
                 B(state_index_-1), B(state_index_), 
                 *preint_imu);

    graph_.add(imu_factor);
   
}

std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> Optimizer::getImuData(uint64_t start_time, uint64_t end_time){
    
    std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> result;
    while (imu_buffer_.size() > 0){
        if (imu_buffer_[0].first < start_time){
            imu_buffer_.pop_front();
            continue;
        }

        if (imu_buffer_[0].first >= end_time || imu_buffer_.size() == 0){
            break;
        }

        result.push_back(imu_buffer_[0]);
        imu_buffer_.pop_front();
    }

    return result;
}

void Optimizer::addImageFactor(std::pair<uint64_t, geometry_msgs::PoseWithCovariance> odometry) // This function signature will need to change depending on how we want to pass features
{
    /* TODO: Add projection factors
     */
}

void Optimizer::optimizationLoop(){
    std::chrono::duration<double> time_span; 
    std::vector<uint64_t> time_stamps; 
    Values result, final_result;

    for (;;){
        if (initialized_){
            t2_ =  std::chrono::high_resolution_clock::now(); 
            std::chrono::duration<double>time_span = t2_ - t1_;
            if(time_span.count()>5){
                final_result = result;
                break;
            }
        }

        if (!initialized_ || image_buffer_.size() == 0){
            continue;    
        }
        t1_ = t2_;
        if (image_buffer_.size() > 1){
            ROS_ERROR_STREAM("WARNING: Backend can't keep up");
        }

        state_index_ ++;

        uint64_t current_frame_time = image_buffer_[0].first;
        time_stamps.push_back(current_frame_time);

        std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data = getImuData(previous_frame_time, current_frame_time);

        if (imu_data.size() != 0){
            addImuFactor(imu_data);
        }

        /*  TODO: Insert projection factors into graph

        */

        prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
        initial_values_.insert(X(state_index_), prop_state_.pose());
        initial_values_.insert(V(state_index_), prop_state_.v());
        initial_values_.insert(B(state_index_), prev_bias_);


        //Runs GN Optimization on factor graph    
        GaussNewtonOptimizer optimizer(graph_, initial_values_);
        result = optimizer.optimize();

        prev_state_ = NavState(result.at<Pose3>(X(state_index_)),
                            result.at<Vector3>(V(state_index_)));

        prev_bias_ = result.at<imuBias::ConstantBias>(B(state_index_));

        imu_preintegrated_ -> resetIntegrationAndSetBias(prev_bias_);

        previous_frame_time = current_frame_time;

        image_buffer_.pop_front();

    }

    // if (LOG_DATA){
    //     NavState current_state;
    //     for (int i = 0; i < state_index_; i++){
    //         current_state = NavState(final_result.at<Pose3>(X(i)),
    //                     final_result.at<Vector3>(V(i)));

    //         std::fstream trajfile;
    //         gtsam::Vector q = current_state.pose().rotation().quaternion();
    //         trajfile.open ("/home/joshua/traj_data/direct_vio.csv",std::ofstream::out | std::ofstream::app);
    //         if (trajfile.is_open()) {  
    //                 trajfile.setf(std::ios_base::fixed);
    //                 trajfile << std::fixed << time_stamps[i] << "," << current_state.pose().x()
    //                     << "," << current_state.pose().y()
    //                     << "," << current_state.pose().z()<<","<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<< std::endl ;
    //                 trajfile.close();
    //         }
    //     }
    // }
}


