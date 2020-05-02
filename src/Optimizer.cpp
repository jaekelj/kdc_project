/*
*  Some sections are taken from GTSAM example: https://github.com/haidai/gtsam/blob/master/examples/ImuFactorsExample.cpp
*/

#include "Optimizer.hpp"

void Optimizer::initializeGraph(uint64_t current_time){


    setParams();
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
}

void Optimizer::addImuFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> data_to_add)
{
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


void Optimizer::addDynamicsFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> data_to_add){
    NonlinearFactorGraph dynamicsFactors_;
    std::vector<int> dynamicsFactorTypes_;

    for (std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>>::iterator it = data_to_add.begin() ; it != data_to_add.end(); ++it)
    {
        Eigen::Matrix<double,5,1> dynamicsMeasurement = it->second;

        // dynamics_preintegrated_->integrateMeasurement(Vector3(imuMeasurement[1],imuMeasurement[2],imuMeasurement[3]),
                                        //    Vector3(imuMeasurement[4],imuMeasurement[5],imuMeasurement[6]),
                                        //    imuMeasurement[0]); // TODO update line for dynamics
    }


    PreintegratedCombDynamicsMeasurements *preint_dynamics = dynamic_cast<PreintegratedCombDynamicsMeasurements*>(dynamics_preintegrated_);
    DynamicsFactor dynamics_factor(X(state_index_-1), V(state_index_-1), X(state_index_), V(state_index_), *preint_dynamics); // TODO check
    graph_.add(dynamics_factor);
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

std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> Optimizer::getDynamicsData(uint64_t start_time, uint64_t end_time){

    std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> result;
    while (dynamics_buffer_.size() > 0){
        if (dynamics_buffer_[0].first < start_time){
            dynamics_buffer_.pop_front();
            continue;
        }

        if (dynamics_buffer_[0].first >= end_time || dynamics_buffer_.size() == 0){
            break;
        }

        result.push_back(dynamics_buffer_[0]);
        dynamics_buffer_.pop_front();
    }

    return result;
}

void Optimizer::setParams(){
    for (int i = 0; i < p_.num_pairs; i++){
        cv::Mat tmp_R;
        cv::Mat tmp_t;

        p_.t_imu_camL[i].copyTo(tmp_t);
        tmp_t = tmp_t.t();
        tmp_t.convertTo(tmp_t, CV_64F);
        p_.R_imu_camL[i].convertTo(tmp_R, CV_64F);

        Rot3 cam_rot_imu_camL(Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(tmp_R.ptr<double>(),tmp_R.rows,tmp_R.cols));
        Point3 cam_trans_imu_camL(Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>>(tmp_t.ptr<double>(),tmp_t.rows, tmp_t.cols));
        P_IMU_CAML_.push_back(Pose3(cam_rot_imu_camL, cam_trans_imu_camL));

        p_.t_imu_camR[i].copyTo(tmp_t);
        tmp_t = tmp_t.t();
        tmp_t.convertTo(tmp_t, CV_64F);
        p_.R_imu_camR[i].convertTo(tmp_R, CV_64F);

        Rot3 cam_rot_imu_camR(Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(tmp_R.ptr<double>(),tmp_R.rows,tmp_R.cols));
        Point3 cam_trans_imu_camR(Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>>(tmp_t.ptr<double>(),tmp_t.rows, tmp_t.cols));
        P_IMU_CAMR_.push_back(Pose3(cam_rot_imu_camR, cam_trans_imu_camR));
    }
}

void Optimizer::addImageFactor(std::pair<uint64_t, geometry_msgs::PoseWithCovariance> odometry)
{
    geometry_msgs::Point p = odometry.second.pose.position;
    geometry_msgs::Quaternion q = odometry.second.pose.orientation;
    Rot3 R( q.w, q.x, q.y, q.z);
    Point3 t( p.x, p.y, p.z);
    Pose3 odom(R,t);
    // Eigen::Map<Matrix6> cov(odometry.second.covariance.data());
    Matrix6 cov = Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor> >(odometry.second.covariance.data());
    noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Covariance(cov*10000);
    BetweenFactor<Pose3> dvo_factor(X(state_index_), X(state_index_-1), odom, noise);
    graph_.add(dvo_factor);

}

void Optimizer::optimizationLoop(){
    std::chrono::duration<double> time_span;
    std::vector<uint64_t> time_stamps;
    Values result, final_result;

       for (;;){
        // std::cout << "buffer size: " << image_buffer_.size() << std::endl;
        if (initialized_){
            // std::cout << "initialized" << std::endl;
            t2_ =  std::chrono::high_resolution_clock::now(); 
            std::chrono::duration<double>time_span = t2_ - t1_;
            // std::cout << "time diff is " << time_span.count() << std::endl;
            if(time_span.count()>5){
                final_result = result;
                break;
            }
        }

        if (!initialized_ || image_buffer_.size() == 0){
            continue;    
        }
        std::cout << "In loop" << std::endl;

        t1_ = t2_;
        if (image_buffer_.size() > 1){
            std::cout << "WARNING: Backend can't keep up" << std::endl;
        }

        state_index_ ++;

        uint64_t current_frame_time = image_buffer_[0].first;
        time_stamps.push_back(current_frame_time);

        std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data = getImuData(previous_frame_time, current_frame_time);

        if (imu_data.size() != 0){
            addImuFactor(imu_data);
        }

        std::vector<std::pair<uint64_t, Eigen::Matrix<double, 5, 1>>> dynamics_data = getDynamicsData(previous_frame_time,current_frame_time);
        if (dynamics_data.size() != 0){
            addDynamicsFactor(dynamics_data);
        }

        //Add DVO factor
        addImageFactor(image_buffer_[0]);

        prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
        initial_values_.insert(X(state_index_), prop_state_.pose());
        initial_values_.insert(V(state_index_), prop_state_.v());
        initial_values_.insert(B(state_index_), prev_bias_);

        // if (state_index_ < 2)
            graph_.print();

        GaussNewtonOptimizer optimizer(graph_, initial_values_);
        result = optimizer.optimize();

        prev_state_ = NavState(result.at<Pose3>(X(state_index_)),
                            result.at<Vector3>(V(state_index_)));

        prev_bias_ = result.at<imuBias::ConstantBias>(B(state_index_));
        imu_preintegrated_ -> resetIntegrationAndSetBias(prev_bias_);
        previous_frame_time = current_frame_time;
        image_buffer_.pop_front();

        std::cout << "Through optimization loop" << std::endl;
    }
}
