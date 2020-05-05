#include <Parameters.hpp>

void Parameters::readConfig(std::string file_path){
    cv::FileStorage config_file(file_path, cv::FileStorage::READ);
    if (!config_file.isOpened())
    {
        ROS_ERROR_STREAM("ERROR: Wrong path to settings");
    }
    // Number of stereo camera pairs
    num_pairs = config_file["num_pairs"];

    // Image size
    col = config_file["image_width"];
    row = config_file["image_height"];

    // Camera intrinsics
    for (auto item : config_file["intrinsics"]){
        cv::Mat tmpL, tmpR;
        item["left"] >> tmpL;
        item["right"] >> tmpR;

        tmpL.convertTo(tmpL,CV_32F);
        tmpR.convertTo(tmpR,CV_32F);

        cv::Mat K = cv::Mat::zeros(3,3,CV_32F);
        K.at<float>(2,2) = 1;

        K.at<float>(0,0) = tmpL.at<float>(0,0);
        K.at<float>(1,1) = tmpL.at<float>(0,1);
        K.at<float>(0,2) = tmpL.at<float>(0,2);
        K.at<float>(1,2) = tmpL.at<float>(0,3);
        intrinsics_L.push_back(K);
        Eigen::VectorXd Kl(5);
        Kl << tmpL.at<float>(0,0), tmpL.at<float>(0,1), 0, tmpL.at<float>(0,2), tmpL.at<float>(0,3);
        intrinsics_vec_L.push_back(Kl);

        K.at<float>(0,0) = tmpR.at<float>(0,0);
        K.at<float>(1,1) = tmpR.at<float>(0,1);
        K.at<float>(0,2) = tmpR.at<float>(0,2);
        K.at<float>(1,2) = tmpR.at<float>(0,3);
        intrinsics_R.push_back(K);
        Eigen::VectorXd Kr(5);
        Kr << tmpR.at<float>(0,0), tmpR.at<float>(0,1), 0, tmpR.at<float>(0,2), tmpR.at<float>(0,3);
        intrinsics_vec_R.push_back(Kr);
    }

    // Camera extrinsics
    for (auto item : config_file["extrinsics"]){
        cv::Mat tmpL, tmpR;
        item["left"] >> tmpL;
        item["right"] >> tmpR;

        tmpL.convertTo(tmpL,CV_32F);
        tmpR.convertTo(tmpR,CV_32F);

        T_camL_imu.push_back(tmpL);
        R_camL_imu.push_back(tmpL(cv::Rect(0,0,3,3)));
        t_camL_imu.push_back(tmpL(cv::Rect(3,0,1,3)));
        R_camR_imu.push_back(tmpR(cv::Rect(0,0,3,3)));
        t_camR_imu.push_back(tmpR(cv::Rect(3,0,1,3)));

        int curr_index = R_camL_imu.size() - 1;
        cv::Mat T_camL_camR = tmpL*tmpR.inv();
        R_camL_camR.push_back(T_camL_camR(cv::Rect(0,0,3,3)));
        t_camL_camR.push_back(T_camL_camR(cv::Rect(3,0,1,3)));
        t_camL_camR_hat.push_back(buildSkewSym(t_camL_camR[curr_index]));
        left_cam_extrinsics.push_back(intrinsics_L[curr_index]*tmpL(cv::Rect(0,0,4,3)));
        right_cam_extrinsics.push_back(intrinsics_R[curr_index]*tmpR(cv::Rect(0,0,4,3)));
        baselines.push_back(cv::norm(t_camL_camR[curr_index]));

        tmpL = tmpL.inv();
        tmpR = tmpR.inv();

        T_imu_camL.push_back(tmpL);
        R_imu_camL.push_back(tmpL(cv::Rect(0,0,3,3)));
        t_imu_camL.push_back(tmpL(cv::Rect(3,0,1,3)));
        R_imu_camR.push_back(tmpR(cv::Rect(0,0,3,3)));
        t_imu_camR.push_back(tmpR(cv::Rect(3,0,1,3)));
    }

    // Epipolar geometry
    for (int i = 0; i < num_pairs; i++){
        cv::Mat current_F = intrinsics_L[i].inv().t() * t_camL_camR_hat[i] * R_camL_camR[i] * intrinsics_R[i].inv();

        cv::Mat scaling_mat = cv::Mat::ones(3,3,CV_32F);
        scaling_mat.at<float>(0,0) = 1 / std::max(row, col);
        scaling_mat.at<float>(1,1) = 1 / std::max(row, col);
        current_F = scaling_mat * current_F *scaling_mat;

        F.push_back(current_F);
    }

    config_file["T_W_imu"] >> T_W_imu;

    // Camera distortion coeffs
    for (auto item : config_file["distortion_coeffs"]){
        cv::Mat tmpL, tmpR;

        item["left"] >> tmpL;
        item["right"] >> tmpR;

        tmpL.convertTo(tmpL,CV_32F);
        tmpR.convertTo(tmpR,CV_32F);

        distortion_coeffs_L.push_back(tmpL);
        distortion_coeffs_R.push_back(tmpR);
    }

    // Topic names
    imu_topic = (std::string) config_file["imu_topic"];
    motors_topic = (std::string) config_file["motors_topic"];
    visualize_topic = (std::string) config_file["visulize_pub_topic"];
    for (auto item : config_file["image_topics"]){
        left_image_topics.push_back((std::string) item["left"]);
        right_image_topics.push_back((std::string) item["right"]);
    }

    // Optimization params
    window_size = config_file["lag_size"];
    preint_cov = config_file["preint_cov"];
    preint_bias_cov = config_file["preint_bias_cov"];
    max_iter = config_file["max_iter"];
    abs_tol = config_file["abs_tol"];
    rel_tol = config_file["rel_tol"];
    imurate = static_cast<int> (config_file["propagateAtImuRate"])!=0;
    keyframe_spacing = config_file["keyframe_spacing"];

    // Imu params
    sigma_a = config_file["sigma_a"];
    sigma_ba = config_file["sigma_ba"];
    sigma_g = config_file["sigma_g"];
    sigma_bg = config_file["sigma_bg"];

    // Camera noise params
    sigma_pix = config_file["sigma_pixel"];

    // Debug features
    add_features = config_file["add_features"];

    // Front end parameters
    ransac_thresh = config_file["ransac_threshold"];
    grid_col = config_file["grid_col"];
    grid_row = config_file["grid_row"];
    grid_max_features = config_file["grid_max_feature_num"];
    grid_min_features = config_file["grid_min_feature_num"];
    grid_height = row / grid_row;
    grid_width = col / grid_col;

    pyramid_levels = config_file["pyramid_levels"];
    patch_size = config_file["patch_size"];
    min_dist = config_file["mind_dist"];
    num_features = config_file["num_features"];

    config_file.release();
}

cv::Mat Parameters::buildSkewSym(const cv::Mat& vec){
    cv::Mat result = cv::Mat::zeros(3,3,CV_32F);
    result.at<float>(0,1) = -vec.at<float>(0,2);
    result.at<float>(0,2) = vec.at<float>(0,1);
    result.at<float>(1,2) = -vec.at<float>(0,0);
    result.at<float>(1,0) = vec.at<float>(0,2);
    result.at<float>(2,0) = -vec.at<float>(0,1);
    result.at<float>(2,1) = vec.at<float>(0,0);

    return result;
}