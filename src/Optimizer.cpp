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
    // prior_pose.
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

    dynamics_preintegrated_ = new PreintegratedCombDynamicsMeasurements();

    prev_state_ = NavState(prior_pose, prior_velocity);
    prev_bias_ = prior_bias;
    prop_state_ = prev_state_;

    initialized_ = true;

    t1_ =  std::chrono::high_resolution_clock::now();
    t2_ =  std::chrono::high_resolution_clock::now();
}

void Optimizer::addImuFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> data_to_add)
{
    // std::cout << "adding imu factor" << std::endl;
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


void Optimizer::addDynamicsFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> dynamics_data_to_add,
                                  std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data_to_add){
    NonlinearFactorGraph dynamicsFactors_;
    std::vector<int> dynamicsFactorTypes_;

    std::cout << "adding dynamics factor with " << dynamics_data_to_add.size() << " dynamics measurements and " << imu_data_to_add.size() << " imu measurements" << std::endl;
    for (std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>>::iterator it = dynamics_data_to_add.begin() ; it != dynamics_data_to_add.end(); ++it)
    {
        Eigen::Matrix<double,5,1> dynamicsMeasurement = it->second;

        // get IMU measurement from right before dynamics measurement  TODO interpolate
        auto dynamicsTime = it->first;
        // TODO optimize loop to not start from scratch each time
        std::pair<uint64_t,Eigen::Matrix<double,7,1>> prev_imu = *imu_data_to_add.begin();
        std::pair<uint64_t,Eigen::Matrix<double,7,1>> next_imu;

        for (auto it = imu_data_to_add.begin(); it != imu_data_to_add.end(); ++it) {
          if (it->first >= dynamicsTime){
            next_imu = *it;
            break;
          }
        }

        // TODO interpolate IMU


        // Calculate thrust
        // Blackbird
        double gamma = 0.01;
        double ct = 2.081e-08; // from in flight calibration
        double m = 0.915; // TODO remove hardcode

        double T = 0;
        for (int i = 1; i < 5; ++i) {
          T += std::pow(dynamicsMeasurement[i],2)*gamma;
        }

        T = T*ct/m;

        Eigen::Vector3d T_b(0, 0, T);

        // dt
        double dt = dynamicsMeasurement[0]; // previously calculated dt

        dynamics_preintegrated_->integrateMeasurement(Vector3(T_b[0],T_b[1],T_b[2]), Vector3(prev_imu.second[4],prev_imu.second[5],prev_imu.second[6]), dt);
    }

    PreintegratedCombDynamicsMeasurements *preint_dynamics = dynamic_cast<PreintegratedCombDynamicsMeasurements*>(dynamics_preintegrated_);
    DynamicsFactor dynamics_factor(X(state_index_-1), V(state_index_-1), X(state_index_), V(state_index_), *preint_dynamics); // TODO check
    graph_.add(dynamics_factor);
    dynamics_preintegrated_->resetIntegration();
}

void Optimizer::addRotorsFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> rotors_data_to_add,
                                  std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data_to_add){
    NonlinearFactorGraph rotorsFactors_;
    std::vector<int> rotorsFactorTypes_;

    // std::cout << "adding dynamics factor with " << dynamics_data_to_add.size() << " dynamics measurements and " << imu_data_to_add.size() << " imu measurements" << std::endl;
    for (std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>>::iterator it = rotors_data_to_add.begin() ; it != rotors_data_to_add.end(); ++it)
    {
        Eigen::Matrix<double,5,1> rotorsMeasurement = it->second;

        // get IMU measurement from right before dynamics measurement  TODO interpolate
        auto rotorsTime = it->first;
        // TODO optimize loop to not start from scratch each time
        std::pair<uint64_t,Eigen::Matrix<double,7,1>> prev_imu = *imu_data_to_add.begin();
        std::pair<uint64_t,Eigen::Matrix<double,7,1>> next_imu;

        for (auto it = imu_data_to_add.begin(); it != imu_data_to_add.end(); ++it) {
          if (it->first >= rotorsTime){
            next_imu = *it;
            break;
          }
        }

        // TODO interpolate IMU


        // Calculate thrust
        // RotorS sim
        double m = 1.52;
        double ct = 8.549e-6;
        double gamma = 1.0;

        double T = 0;
        for (int i = 1; i < 5; ++i) {
          T += std::pow(rotorsMeasurement[i],2)*gamma;
        }

        T = T*ct/m;

        Eigen::Vector3d T_b(0, 0, T);

        // dt
        double dt = rotorsMeasurement[0]; // previously calculated dt

        dynamics_preintegrated_->integrateMeasurement(Vector3(T_b[0],T_b[1],T_b[2]), Vector3(prev_imu.second[4],prev_imu.second[5],prev_imu.second[6]), dt);
    }

    PreintegratedCombDynamicsMeasurements *preint_dynamics = dynamic_cast<PreintegratedCombDynamicsMeasurements*>(dynamics_preintegrated_);
    DynamicsFactor dynamics_factor(X(state_index_-1), V(state_index_-1), X(state_index_), V(state_index_), *preint_dynamics); // TODO check
    graph_.add(dynamics_factor);
    dynamics_preintegrated_->resetIntegration();
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
        // std::cout << " end time is " << (long long int) end_time - (long long int) start_time << " message time is " << (long long int) dynamics_buffer_[0].first - (long long int) start_time<< std::endl;
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

std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> Optimizer::getRotorsData(uint64_t start_time, uint64_t end_time){

    std::vector<std::pair<uint64_t,Eigen::Matrix<double,5,1>>> result;
    while (rotors_buffer_.size() > 0){
        // std::cout << " end time is " << (long long int) end_time - (long long int) start_time << " message time is " << (long long int) dynamics_buffer_[0].first - (long long int) start_time<< std::endl;
        if (rotors_buffer_[0].first < start_time){
            rotors_buffer_.pop_front();
            continue;
        }

        if (rotors_buffer_[0].first >= end_time || rotors_buffer_.size() == 0){
            break;
        }

        result.push_back(rotors_buffer_[0]);
        rotors_buffer_.pop_front();
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
    // std::cout << "adding image factor" << std::endl;
    geometry_msgs::Point p = odometry.second.pose.position;
    geometry_msgs::Quaternion q = odometry.second.pose.orientation;
    Rot3 R( q.w, q.x, q.y, q.z);
    Point3 t( p.x, p.y, p.z);
    if (std::isnan(p.x)){
        R = Rot3(1,0,0,0);
        t = Point3(0,0,0);
        std::cout << "got nan" << std::endl;
    }
    // std::cout << "measurement is " << std::isnan(p.x) << std::endl;
    Pose3 odom(R,t);
    // Eigen::Map<Matrix6> cov(odometry.second.covariance.data());
    Matrix6 cov = 5e-6*Eigen::MatrixXd::Identity(6,6);

    noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Covariance(cov);
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
        else{
            std::cout << "GOT 0 IMU MEASUREMENTS!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        }

        // std::cout << "Dynamics buffer size is " << rotors_buffer_.size() << std::endl;
        // std::vector<std::pair<uint64_t, Eigen::Matrix<double, 5, 1>>> dynamics_data = getDynamicsData(previous_frame_time,current_frame_time);
        // if (dynamics_data.size() != 0 && imu_data.size() != 0){
        //     addDynamicsFactor(dynamics_data, imu_data);
        // }
        // else{
        //     std::cout << "Got 0 dynamics measurements." << std::endl;
        // }

        // Rotors factor
        std::cout << "Rotors buffer size is " << rotors_buffer_.size() << std::endl;
        std::vector<std::pair<uint64_t, Eigen::Matrix<double, 5, 1>>> rotors_data = getRotorsData(previous_frame_time,current_frame_time);
        if (rotors_data.size() != 0 && imu_data.size() != 0){
            addRotorsFactor(rotors_data, imu_data);
        }

        //Add DVO factor
        addImageFactor(image_buffer_[0]);

        prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
        initial_values_.insert(X(state_index_), prop_state_.pose());
        initial_values_.insert(V(state_index_), prop_state_.v());
        initial_values_.insert(B(state_index_), prev_bias_);

        // initial_values_.print();
        // graph_.print();

        GaussNewtonOptimizer optimizer(graph_, initial_values_);
        result = optimizer.optimize();

        // Vector3 prior_velocity(0,0,0);
        // prev_state_ = NavState(result.at<Pose3>(X(state_index_)), prior_velocity);
        prev_state_ = NavState(result.at<Pose3>(X(state_index_)),
                            result.at<Vector3>(V(state_index_)));



        prev_bias_ = result.at<imuBias::ConstantBias>(B(state_index_));
        // imuBias::ConstantBias prior_bias; //Assume zeros bias
        // prev_bias_ = prior_bias;

        imu_preintegrated_ -> resetIntegrationAndSetBias(prev_bias_);
        previous_frame_time = current_frame_time;
        image_buffer_.pop_front();

        nav_msgs::Odometry new_odom_msg;
        new_odom_msg.header.frame_id = "/world";
        new_odom_msg.child_frame_id = "/baselink";
        new_odom_msg.pose.pose.position.x = prev_state_.position().x();
        new_odom_msg.pose.pose.position.y = prev_state_.position().y();
        new_odom_msg.pose.pose.position.z = prev_state_.position().z();

        new_odom_msg.pose.pose.orientation.x = prev_state_.pose().rotation().toQuaternion().x();
        new_odom_msg.pose.pose.orientation.y = prev_state_.pose().rotation().toQuaternion().y();
        new_odom_msg.pose.pose.orientation.z = prev_state_.pose().rotation().toQuaternion().z();
        new_odom_msg.pose.pose.orientation.w = prev_state_.pose().rotation().toQuaternion().w();

        odom_buffer_.push_back(new_odom_msg);
    }
}
