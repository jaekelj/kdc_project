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

void Optimizer::addImuFactor(std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> data_to_add){
    NonlinearFactorGraph imuFactors_;
    std::vector<int> imuFactorTypes_;

    for (std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>>::iterator it = data_to_add.begin() ; it != data_to_add.end(); ++it)
    {
        Eigen::Matrix<double,7,1> imuMeasurement = it->second;
        imu_preintegrated_->integrateMeasurement(Vector3(imuMeasurement[1],imuMeasurement[2],imuMeasurement[3]),
                                           Vector3(imuMeasurement[4],imuMeasurement[5],imuMeasurement[6]),
                                           imuMeasurement[0]);
    }


    PreintegratedCombinedMeasurements *preint_imu = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_); // analagous

    CombinedImuFactor imu_factor(X(state_index_-1), V(state_index_-1),
                 X(state_index_  ), V(state_index_  ),
                 B(state_index_-1), B(state_index_),
                 *preint_imu); // analogous

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

void Optimizer::addProjectionFactor(const std::vector<FeatureHandler::BackendFeature>& features){
        NonlinearFactorGraph visionFactors_;
        Values visionVariables_;
        std::vector<int> visionFactorTypes_;

        NonlinearFactorGraph landmarkPriorFactors_;
        Values landmarkPriorVariables_;
        std::vector<int> landmarkPriorFactorTypes_;

        const noiseModel::Isotropic::shared_ptr cam_noise = noiseModel::Isotropic::Sigma(4,p_.sigma_pix);

        SharedNoiseModel cam_noisemodel = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(2.37), cam_noise);

        int total_counter = 0;
        int accepted_counter = 0;
        for(auto const& measurement : features)
        {
            // First assume both camera has seen the landmark
            bool stereo_obs = true;
            Eigen::Vector2d left_p((double) measurement.coordinate_L_.x, (double) measurement.coordinate_L_.y);
            Eigen::Vector2d right_p((double) measurement.coordinate_R_.x, (double) measurement.coordinate_R_.y);
            uint64_t id = measurement.fid_;
            int pair_id = 0;

            if(std::isnan(right_p[0])){ // if right point is valid
                    stereo_obs = false;
                    continue;
            }

            if (map_.find(id) == map_.end())
            {
                Landmark* landmark = new Landmark();
                landmark->id = id;
                Eigen::Vector4d measurement(left_p[0],left_p[1],right_p[0],right_p[1]);
                landmark->isStereo_= true;
                landmark->factorAdded = false;

                map_.insert(std::pair<uint64_t,Landmark*>(id,landmark));
                stateMap_.at(state_index_).insert(std::make_pair(id,landmark));
            }

            else
            {
                CameraSet<PinholeCamera<Cal3_S2>> monoCameras;
                PinholeCamera<Cal3_S2>::MeasurementVector monoMeasured;

                // Using propagated state for feature prediction
                const Pose3 leftPose = prev_state_.pose().compose(P_IMU_CAML_[pair_id]);
                const Pose3 rightPose = prev_state_.pose().compose(P_IMU_CAMR_[pair_id]);
                const Cal3_S2 monoCal_left(p_.intrinsics_vec_L[pair_id]);
                const Cal3_S2 monoCal_right(p_.intrinsics_vec_R[pair_id]);
                const PinholeCamera<Cal3_S2> leftCamera_i(leftPose, monoCal_left);
                const PinholeCamera<Cal3_S2> rightCamera_i(rightPose, monoCal_right);

                // std::cout << "left cam is " << p_.intrinsics_vec_L[pair_id] << std::endl;
                // std::cout << "right cam is " << p_.intrinsics_vec_R[pair_id] << std::endl;
                // std::cout << "left point is " << Point2(left_p[0],left_p[1]) << std::endl;
                // std::cout << "right point is " << Point2(right_p[0],right_p[1]) << std::endl;

                monoCameras.push_back(leftCamera_i);
                monoMeasured.push_back(Point2(left_p[0],left_p[1]));
                monoCameras.push_back(rightCamera_i);
                monoMeasured.push_back(Point2(right_p[0],right_p[1]));

                TriangulationParameters triangulationParam;
                triangulationParam.landmarkDistanceThreshold = 15;
                triangulationParam.dynamicOutlierRejectionThreshold = 1;
                triangulationParam.rankTolerance = 1e-14;
                TriangulationResult result_ = gtsam::triangulateSafe(monoCameras, monoMeasured, triangulationParam);

                Point3 pos_prior;
                total_counter++;
                if (!result_)
                {
                    // std::cout << "result is " << result_ << std::endl;
                    // std::cout << "left cam is " << P_IMU_CAML_[pair_id] << std::endl;
                    // // leftCamera_i.print();
                    // std::cout << "right cam is " << P_IMU_CAMR_[pair_id] << std::endl;
                    // rightCamera_i.print();
                    continue;
                }
                else
                {
                    accepted_counter++;
                    pos_prior = *result_;
                }

                // std::cout << "adding point " << id << std::endl;
                Landmark *curr_landmark = map_.at(id);
                stateMap_.at(state_index_).insert(std::pair<int,Landmark*>(id,curr_landmark));
                Eigen::Vector4d measurement(left_p[0],left_p[1],right_p[0],right_p[1]);
                if (!curr_landmark->factorAdded && curr_landmark->isStereo_)
                {
                    curr_landmark->observations.insert(std::make_pair(state_index_,measurement));
                    if (curr_landmark->observations.size()>1)
                    {
                        initial_values_.insert(L(id), pos_prior);
                        bool added_prior = false;
                        for(auto const &obs : curr_landmark->observations)
                        {
                            if (stateMap_.find(obs.first) == stateMap_.end())
                                 continue;
                            Eigen::Vector4d measurement = obs.second;
                            StereoLandmarkFactor stereoFactorUnrect = StereoLandmarkFactor(X(obs.first),L(id),cam_noisemodel, P_IMU_CAML_[pair_id], P_IMU_CAMR_[pair_id], p_.intrinsics_vec_L[pair_id], p_.intrinsics_vec_R[pair_id], Point2(measurement[0], measurement[1]), Point2(measurement[2], measurement[3]));
                            graph_.add(stereoFactorUnrect);
                            visionFactorTypes_.push_back(FactorType::Projection);
                            if (!added_prior){
                                noiseModel::Diagonal::shared_ptr landmarkPriorModel = noiseModel::Diagonal::Sigmas(Vector3(10,10,10));
                                gtsam::PriorFactor<Point3> landmark_prior = gtsam::PriorFactor<Point3>(L(id),pos_prior,landmarkPriorModel);
                                graph_.add(landmark_prior);
                                landmarkPriorFactorTypes_.push_back(FactorType::Prior);
                                added_prior = true;
                            }
                        }
                        curr_landmark->factorAdded = true;
                    }
                }

                else if (curr_landmark->factorAdded)
                {
                    gtsam::StereoLandmarkFactor stereoFactorUnrect = gtsam::StereoLandmarkFactor(X(state_index_),L(id),cam_noisemodel,P_IMU_CAML_[pair_id], P_IMU_CAMR_[pair_id], p_.intrinsics_vec_L[pair_id], p_.intrinsics_vec_R[pair_id], Point2(left_p[0], left_p[1]), Point2(right_p[0], right_p[1]));
                    graph_.add(stereoFactorUnrect);
                    visionFactorTypes_.push_back(FactorType::Projection);
                    Eigen::Vector4d measurement(left_p[0],left_p[1],right_p[0],right_p[1]);
                    curr_landmark->observations.insert(std::make_pair(state_index_,measurement));
                }
            }
        }
    // std::cout << "Accepted " << accepted_counter << " of " << total_counter << " features" << std::endl;
}

void Optimizer::optimizationLoop(){
    std::chrono::duration<double> time_span;
    std::vector<uint64_t> time_stamps;
    Values result, final_result;

    std::cout << "starting optimization" << std::endl;

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
        std::map<int, Landmark*> new_landmark_map;
        stateMap_.insert(std::make_pair(state_index_,new_landmark_map));

        uint64_t current_frame_time = image_buffer_[0].first;
        time_stamps.push_back(current_frame_time);

        std::vector<std::pair<uint64_t,Eigen::Matrix<double,7,1>>> imu_data = getImuData(previous_frame_time, current_frame_time);

        if (imu_data.size() != 0){
            addImuFactor(imu_data);
        }

        /*  TODO: Insert projection factors into graph
        */
        addProjectionFactor(image_buffer_[0].second);

        prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
        initial_values_.insert(X(state_index_), prop_state_.pose());
        initial_values_.insert(V(state_index_), prop_state_.v());
        initial_values_.insert(B(state_index_), prev_bias_);

        //Runs GN Optimization on factor graph
        GaussNewtonOptimizer optimizer(graph_, initial_values_);
        result = optimizer.optimize();

        prev_state_ = NavState(result.at<Pose3>(X(state_index_)), result.at<Vector3>(V(state_index_)));

        prev_bias_ = result.at<imuBias::ConstantBias>(B(state_index_));

        imu_preintegrated_ -> resetIntegrationAndSetBias(prev_bias_);

        previous_frame_time = current_frame_time;

        image_buffer_.pop_front();
    }
}
