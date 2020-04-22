#include <FeatureHandler.hpp>

std::vector<std::vector<FeatureHandler::BackendFeature>> FeatureHandler::fundamentalMatrixRANSAC(std::vector<std::vector<Feature>>& features, const sensor_msgs::ImageConstPtr& cam0, const sensor_msgs::ImageConstPtr& cam1)
{
    cv_bridge::CvImageConstPtr cam0_ptr = cv_bridge::toCvCopy(cam0, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr cam1_ptr = cv_bridge::toCvCopy(cam1, sensor_msgs::image_encodings::MONO8);

    std::vector<std::vector<FeatureHandler::BackendFeature> > inliers_to_backend(p_.num_pairs);

    for (int pair_id = 0; pair_id < p_.num_pairs; pair_id++){
        std::vector<std::vector<cv::Point2f>> points_prev_undistort_l(0);
        std::vector<std::vector<cv::Point2f>> points_curr_undistort_l(0);
        std::vector<std::vector<cv::Point2f>> points_prev_undistort_r(0);
        std::vector<std::vector<cv::Point2f>> points_curr_undistort_r(0);

        std::vector<cv::Point2f> distorted_points_prev_l(0);
        std::vector<cv::Point2f> distorted_points_curr_l(0);
        std::vector<cv::Point2f> distorted_points_prev_r(0);
        std::vector<cv::Point2f> distorted_points_curr_r(0);

        cv::Mat prev_points_3d_hom;  
        cv::Mat curr_points_3d_hom;  

        int features_in_cam0 = 0;

        bool empty = false;
        std::vector<std::pair<int,int>> inliers_overall;

        empty = false;
    
        for (const auto& feature : features[pair_id]) {
            if (feature_map_.find(feature.fid_) == feature_map_.end()){
                feature_map_.insert(std::make_pair(feature.fid_, max_ransac_fid_));
                max_ransac_fid_++;
            }
            distorted_points_prev_l.push_back(feature.prev_coordinate_L_);
            distorted_points_curr_l.push_back(feature.coordinate_L_);
            distorted_points_prev_r.push_back(feature.prev_coordinate_R_);
            distorted_points_curr_r.push_back(feature.coordinate_R_);
        }
        features_in_cam0 = distorted_points_curr_l.size();
        if (features_in_cam0 == 0){
            empty = true;
        }

        //Undistort points        
        points_prev_undistort_l.push_back(std::vector<cv::Point2f>(distorted_points_prev_l.size()));
        points_curr_undistort_l.push_back(std::vector<cv::Point2f>(distorted_points_curr_l.size()));
        points_prev_undistort_r.push_back(std::vector<cv::Point2f>(distorted_points_prev_r.size()));
        points_curr_undistort_r.push_back(std::vector<cv::Point2f>(distorted_points_curr_r.size()));


        if (empty){
            distorted_points_prev_l.clear();
            distorted_points_curr_l.clear();
            distorted_points_prev_r.clear();
            distorted_points_curr_r.clear();
            continue;
        }

        cv::Mat partial_prev_points_3d;  
        cv::Mat partial_curr_points_3d;  

        // std::cout << left_cam_extrinsics[i] << std::endl;
        undistort(distorted_points_prev_l,points_prev_undistort_l[0],pair_id,true);
        undistort(distorted_points_curr_l,points_curr_undistort_l[0],pair_id,true);
        undistort(distorted_points_prev_r,points_prev_undistort_r[0],pair_id,false);
        undistort(distorted_points_curr_r,points_curr_undistort_r[0],pair_id,false);

        // std::vector<uchar> status_boundary(distorted_points_prev_l.size());

        std::vector<uchar> status_L;
        std::vector<uchar> status_R;

        cv::findFundamentalMat(points_prev_undistort_l[0], points_curr_undistort_l[0] ,cv::FM_RANSAC, 1, 0.99, status_L);
        cv::findFundamentalMat(points_prev_undistort_r[0], points_curr_undistort_r[0] ,cv::FM_RANSAC, 1, 0.99, status_R);  

        for (int j = 0; j < status_L.size(); j++){
            if (status_L[j] == 1 && status_R[j] == 1){
                if(inBorder(points_curr_undistort_l[0][j]) && inBorder(points_curr_undistort_r[0][j])){
                    inliers_overall.push_back(std::make_pair(j,0));
                }
            }
        }

        if(inliers_overall.size() == 0){
            return inliers_to_backend;
        }

        int cam_num, feature_num;
        for (int i = 0; i < inliers_overall.size(); i++)
        {
            cam_num = inliers_overall[i].second;
            feature_num = inliers_overall[i].first;

            BackendFeature new_feat(points_curr_undistort_l[cam_num][feature_num],points_curr_undistort_r[cam_num][feature_num],feature_map_[features[cam_num][feature_num].fid_]);
            inliers_to_backend[cam_num].push_back(new_feat);
        }

        for (int i = 0; i < p_.num_pairs; i++)
            std::cout << "Before RANSAC in pair " << i << ": " << features[i].size() << " features. After RANSAC: " << inliers_to_backend[i].size() << " features. " << cam0->header.stamp.toNSec() << std::endl;

        int j = 0;
        for (int i = 0; i < p_.num_pairs; i++)
        {
            for (const auto &feature : features[i])
            {
                bool new_feat = false;
                if (inliers_overall[j].second != i)
                {
                    break;
                }

                int feature_num = inliers_overall[j].first;

                if (j < inliers_overall.size()){
                    if (features[i][feature_num].fid_ == feature.fid_)
                    {
                        new_feat = true;
                        j++;
                    }
                    else
                    {
                        feature_map_.insert(std::make_pair(feature.fid_, max_ransac_fid_));
                        max_ransac_fid_++;
                    }
                }
                else
                {
                    feature_map_.insert(std::make_pair(feature.fid_, max_ransac_fid_));
                    max_ransac_fid_++;
                }
            }            
        }
    }

    publishFeatureImage(inliers_to_backend[0], cam0_ptr->image, cam1_ptr->image);
    return inliers_to_backend;

}

void FeatureHandler::setPrevImage(const cv::Mat& img)
{
    prev_img_L_[0] = img;
}


void FeatureHandler::undistort(const std::vector<cv::Point2f> &original_points, std::vector<cv::Point2f> &undistorted_points, int pair_id, bool l_cam)
{
    if (l_cam)
    {
        cv::undistortPoints(original_points, undistorted_points, p_.intrinsics_L[pair_id], p_.distortion_coeffs_L[pair_id], cv::noArray(), p_.intrinsics_L[pair_id]);
    }
    else
    {
        cv::undistortPoints(original_points, undistorted_points, p_.intrinsics_R[pair_id], p_.distortion_coeffs_R[pair_id], cv::noArray(), p_.intrinsics_R[pair_id]);
    }
}

void FeatureHandler::initializeFeatures(const cv::Mat &camL_img, const cv::Mat &camR_img, int pair_id)
{

    std::vector<cv::Point2f> corners_L;
    std::vector<cv::Point2f> corners_R;
    std::vector<uchar> status;

    std::map<int, std::vector<Feature>> feat_map;
    tracked_grid_features_.push_back(feat_map);
    cv::goodFeaturesToTrack(camL_img, corners_L, p_.num_features, 0.05, p_.min_dist); //TODO: Change quality to variable
    corners_R = corners_L;
    stereoMatch(camL_img, camR_img, corners_L, corners_R, status, pair_id);

    std::vector<unsigned int> ids;
    std::vector<unsigned int> lifetimes;
    std::vector<unsigned int> pair_ids;
    std::vector<cv::Point2f> coordinates_L, coordinates_R;
    std::vector<int> grid_ids;

    unsigned int counter = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            counter++;
            Feature feature;
            feature.coordinate_L_ = corners_L[i];
            feature.coordinate_R_ = corners_R[i];
            feature.fid_ = counter + max_id_;
            feature.lifetime_ = 1;
            feature.pair_id_ = pair_id;
            tracked_grid_features_[pair_id][assignGridId(corners_L[i])].push_back(feature);
        }
    }
    max_id_ += counter;
    prev_img_L_.push_back(camL_img);
    prev_img_R_.push_back(camR_img);
}

void FeatureHandler::stereoMatch(
    const cv::Mat &camL_img,
    const cv::Mat &camR_img,
    std::vector<cv::Point2f> &pts_imgL,
    std::vector<cv::Point2f> &pts_imgR,
    std::vector<uchar> &status,
    int pair_id)
{

    if (pts_imgL.size() == 0)
        return;

    std::vector<float> err;
    pts_imgR = pts_imgL;
    cv::calcOpticalFlowPyrLK(camL_img, camR_img, pts_imgL, pts_imgR, status, err, cv::Size(p_.patch_size, p_.patch_size), p_.pyramid_levels, cv::TermCriteria(), cv::OPTFLOW_USE_INITIAL_FLOW);

    int counter = 0;
    for (int i = 0; i < pts_imgR.size(); i++)
    {
        if (status[i] == 0)
            continue;
        if (!inBorder(pts_imgR[i]))
            status[i] = 0;
        if (status[i]) counter++;
    }

    if (pts_imgL.size() == 0 || pts_imgR.size() == 0){
        return;
    }

    // // Remove outliers accoirding to the known E matrix
    // std::vector<cv::Point2f> pts_imgL_undistorted(0);
    // std::vector<cv::Point2f> pts_imgR_undistorted(0);

    // cv::undistortPoints(pts_imgL, pts_imgL_undistorted, p_.intrinsics_L[pair_id], p_.distortion_coeffs_L[pair_id], p_.intrinsics_L[pair_id]);
    // cv::undistortPoints(pts_imgR, pts_imgR_undistorted, p_.intrinsics_R[pair_id], p_.distortion_coeffs_R[pair_id], p_.intrinsics_R[pair_id]);

    // float episilon = 10.0;

    // cv::Mat ptL_homo = cv::Mat::ones(3,1,CV_32F);
    // cv::Mat ptR_homo = cv::Mat::ones(3,1,CV_32F);
    // cv::Mat epipolar_line_1;

    // for (int i = 0; i < pts_imgL_undistorted.size(); i++)
    // {
    //     if (status[i] == 0)
    //         continue;
    //     ptL_homo.at<float>(0,0) = pts_imgL_undistorted[i].x;
    //     ptL_homo.at<float>(1,0) = pts_imgL_undistorted[i].y;
    //     ptR_homo.at<float>(0,0) = pts_imgR_undistorted[i].x;
    //     ptR_homo.at<float>(1,0) = pts_imgR_undistorted[i].y;
    //     epipolar_line_1 = p_.F[pair_id] * ptL_homo; 
    //     cv::Mat temp = (ptR_homo.t()*epipolar_line_1);
    //     float distance = fabs(temp.at<float>(0,0)) / sqrt(epipolar_line_1.at<float>(0,0) * epipolar_line_1.at<float>(0,0) + epipolar_line_1.at<float>(1,0) * epipolar_line_1.at<float>(1,0));
    //     if (distance > episilon){
    //         status[i] = 0;
    //     }
    // }
}

void FeatureHandler::temporalMatch(const cv::Mat &camL_img, const cv::Mat &camR_img, uint64_t current_time, int pair_id)
{
    bool compensate_vel = false;

    std::vector<uchar> status;
    std::vector<cv::Point2f> matched_pointsL;
    std::vector<float> err;

    std::vector<unsigned int> ids;
    std::vector<unsigned int> lifetimes;
    std::vector<unsigned int> pair_ids;
    std::vector<cv::Point3f> _3d_points;
    std::vector<cv::Point2f> coordinates_L, coordinates_R;

    for (const auto &it : tracked_grid_features_[pair_id])
    {
        for (auto feature : it.second)
        {
            coordinates_L.push_back(feature.coordinate_L_);
            coordinates_R.push_back(feature.coordinate_R_);
            ids.push_back(feature.fid_);
            lifetimes.push_back(feature.lifetime_);
            pair_ids.push_back(feature.pair_id_);
            _3d_points.push_back(feature._3d_coordinate_);
        }
    }

    tracked_grid_features_[pair_id].clear();

    if (coordinates_L.size() == 0)
        return;

    cv::Matx33f R;

    matched_pointsL = coordinates_L;

    cv::calcOpticalFlowPyrLK(prev_img_L_[pair_id], camL_img, coordinates_L, matched_pointsL, status, err, cv::Size(p_.patch_size, p_.patch_size), p_.pyramid_levels, cv::TermCriteria(), cv::OPTFLOW_USE_INITIAL_FLOW);

    for (size_t i = 0; i < matched_pointsL.size(); i++)
    {
        if (status[i] == 0)
            continue;
        if (!inBorder(matched_pointsL[i]))
            status[i] = 0;
    }

    removeOutliers(status, matched_pointsL);
    removeOutliers(status, coordinates_L);
    removeOutliers(status, coordinates_R);
    removeOutliers(status, ids);
    removeOutliers(status, lifetimes);
    removeOutliers(status, pair_ids);
    removeOutliers(status, _3d_points);

    status.clear();

    std::vector<cv::Point2f> matched_pointsR;

    if (matched_pointsL.size() > 0)
    {
        // std::cout << "tracked " << matched_pointsL.size() << " features temporally" << std::endl;
        stereoMatch(camL_img, camR_img, matched_pointsL, matched_pointsR, status, pair_id);

        removeOutliers(status, matched_pointsL);
        removeOutliers(status, matched_pointsR);
        removeOutliers(status, coordinates_L);
        removeOutliers(status, coordinates_R);
        removeOutliers(status, ids);
        removeOutliers(status, lifetimes);
        removeOutliers(status, pair_ids);
        removeOutliers(status, _3d_points);

        for (size_t i = 0; i < matched_pointsL.size(); i++)
        {
            int gid = assignGridId(matched_pointsL[i]);
            Feature feature;
            feature.prev_coordinate_L_ = coordinates_L[i];
            feature.prev_coordinate_R_ = coordinates_R[i];
            feature.coordinate_L_ = matched_pointsL[i];
            feature.coordinate_R_ = matched_pointsR[i];

            feature.fid_ = ids[i];
            feature.lifetime_ = lifetimes[i] + 1;
            feature.pair_id_ = pair_ids[i];
            feature._3d_coordinate_ = _3d_points[i];
            tracked_grid_features_[pair_id][gid].push_back(feature);
        }
    }
    else{
        std::cout << "tracked 0 features temporally" << std::endl;
    }
}

void FeatureHandler::removeRedundantFeatures()
{
    for (int i = 0; i < p_.num_pairs; i++)
    {
        for (auto &it : tracked_grid_features_[i])
        {
            auto &features_this_grid = it.second;
            if (features_this_grid.size() > p_.grid_max_features)
            {
                // std::cout << "there are " << features_this_grid.size() << " features in this grid and the max is " << p_.grid_max_features << std::endl;
                std::sort(features_this_grid.begin(), features_this_grid.end(), &FeatureHandler::sortByLifetime);
                features_this_grid.erase(features_this_grid.begin() + p_.grid_max_features, features_this_grid.end());
            }
        }
    }
    return;
}

void FeatureHandler::addFeatures(const cv::Mat &camL_img, const cv::Mat &camR_img, int pair_id)
{
    //Create mask
    cv::Mat mask = cv::Mat(p_.row, p_.col, CV_8UC1, 255);
    int n = 0;
    for (const auto &it : tracked_grid_features_[pair_id])
    {
        for (const auto &feature : it.second)
        {
            mask.at<uchar>(feature.coordinate_L_.y, feature.coordinate_R_.x) = 0;
            cv::circle(mask, feature.coordinate_L_, p_.min_dist,0,-1);
            n++;
        }
    }
    if (n >= p_.num_features)
        return;

    std::vector<cv::Point2f> corners_L;
    std::vector<cv::Point2f> corners_R;
    std::vector<uchar> status;

    cv::goodFeaturesToTrack(camL_img, corners_L, p_.num_features - n, 0.05, p_.min_dist, mask); //TODO: change var to param

    if (corners_L.size() == 0)
    {
        return;
    }
    // std::cout << "Found " << corners_L.size() << " potential new features" << std::endl;
    corners_R = corners_L;
    stereoMatch(camL_img, camR_img, corners_L, corners_R, status, pair_id);
    unsigned int counter = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            int gid = assignGridId(corners_L[i]);
            if (tracked_grid_features_[pair_id][gid].size() < p_.grid_min_features)
            {
                counter++;
                Feature feature;
                feature.prev_coordinate_L_ = corners_L[i];
                feature.prev_coordinate_R_ = corners_R[i];
                feature.coordinate_L_ = corners_L[i];
                feature.coordinate_R_ = corners_R[i];
                feature.fid_ = counter + max_id_;
                feature.lifetime_ = 1;
                feature.pair_id_ = pair_id;
                tracked_grid_features_[pair_id][gid].push_back(feature);
            }
        }
    }
    max_id_ += counter;
}

std::vector<FeatureHandler::BackendFeature> FeatureHandler::sendToRANSAC(const sensor_msgs::ImageConstPtr &cam0, const sensor_msgs::ImageConstPtr &cam1)
{

    std::vector<std::vector<Feature>> old_features_master;
    std::vector<std::vector<Feature>> new_features_master;
    for (int i = 0; i < p_.num_pairs; i++)
    {
        std::vector<Feature> old_features, new_features;
        for (const auto &it : tracked_grid_features_[i])
        {
            for (const auto &feature : it.second)
            {
                if (feature.lifetime_ > 1)
                {
                    old_features.push_back(feature);
                }
                else
                {
                    new_features.push_back(feature);
                }
            }
        }
        old_features_master.push_back(old_features);
        new_features_master.push_back(new_features);
        // std::cout << "sending " << old_features.size() << " to RANSAC" << std::endl; 
    }
    return fundamentalMatrixRANSAC(old_features_master,cam0,cam1)[0];
}

void FeatureHandler::publishFeatureImage(const std::vector<FeatureHandler::BackendFeature>& features, const cv::Mat& ptrLeft, const cv::Mat& ptrRight){
    cv::Mat stereo_img(p_.row, p_.col * 2, CV_8UC1);
    cv::Mat left = stereo_img(cv::Range(0, p_.row), cv::Range(0, p_.col));
    ptrLeft.copyTo(left);
    cv::Mat right = stereo_img(cv::Range(0, p_.row), cv::Range(p_.col, 2 * p_.col));
    ptrRight.copyTo(right);
    cv::Mat stereo_color;

    cv::cvtColor(stereo_img, stereo_color, cv::COLOR_GRAY2BGR);
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::Scalar arrow_color = cv::Scalar(0, 255, 0);

    for (auto feat : features)
    {
        cv::circle(stereo_color(cv::Range(0, p_.row), cv::Range(0, p_.row)), feat.coordinate_L_, 4, color, -1);
        cv::circle(stereo_color(cv::Range(0, p_.row), cv::Range(p_.col, 2 * p_.col)), feat.coordinate_R_, 4, color, -1);
        cv::line(stereo_color, feat.coordinate_L_, feat.coordinate_R_ + cv::Point2f(p_.col, 0), cv::Scalar(200, 20, 200));
    }

    cv::imshow("features", stereo_color);
    cv::waitKey(1);
}