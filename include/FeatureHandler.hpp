#ifndef FEATURE_HANDLER_HPP
#define FEATURE_HANDLER_HPP

// STL includes
#include <vector>
#include <random>
#include <unordered_map>
#include <functional>
#include <stdio.h>
#include <queue>
#include <thread>
#include <mutex>
#include <fstream>
#include <stdlib.h>
#include <chrono>

// Boost includes
#include <boost/shared_ptr.hpp>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/highgui/highgui.hpp>

// GTSAM includes
#include <gtsam/geometry/Pose3.h>

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h> 
#include <sensor_msgs/PointCloud2.h>

// Local includes
#include <Parameters.hpp>
#include <utils.hpp>

class FeatureHandler {
public:

    FeatureHandler(const Parameters& p) : p_(p) {
        uint64_t init_time = 0;
        std::vector<float> init_vel = {0,0,0};

        recent_vel = std::make_pair(init_time, init_vel);
        max_ransac_fid_ = 0;
    }

    ~FeatureHandler(){};

    struct BackendFeature{
        cv::Point2f coordinate_L_;
        cv::Point2f coordinate_R_;    
        unsigned int fid_;
        BackendFeature(const cv::Point2f& coordL, const cv::Point2f& coordR, const unsigned int& id ) : coordinate_L_(coordL), coordinate_R_(coordR), fid_(id) {}
    };

    void addFeatures(const cv::Mat & cam0_img, const cv::Mat & cam1_img, int pair_id);

    void temporalMatch(const cv::Mat & cam0_img, const cv::Mat & cam1_img, uint64_t current_time, int pair_id);

    std::vector<FeatureHandler::BackendFeature> sendToRANSAC(const sensor_msgs::ImageConstPtr& cam0, const sensor_msgs::ImageConstPtr& cam1);
    
    void stereoMatch(
        const cv::Mat& cam0_img, 
        const cv::Mat& cam1_img, 
        std::vector<cv::Point2f>& pts_img0, 
        std::vector<cv::Point2f>& pts_img1,
        std::vector<uchar>& status,
        int pair_id);

    void initializeFeatures(const cv::Mat & cam0_img, const cv::Mat & cam1_img, int pair_id);

    void setPrevImage(const cv::Mat& img);

    void removeRedundantFeatures();

private:

    struct Feature {
        cv::Point2f coordinate_L_;
        cv::Point2f coordinate_R_;
        cv::Point2f prev_coordinate_L_;
        cv::Point2f prev_coordinate_R_;
        cv::Point3f _3d_coordinate_;
        unsigned int fid_;
        unsigned int lifetime_; 
        unsigned int pair_id_;
    };

    static bool sortByLifetime(const Feature& F1, const Feature& F2) {
        return F1.lifetime_ > F2.lifetime_;
    }

    template <typename T>
    void removeOutliers(std::vector<uchar>& status, std::vector<T>& inputs) {
        int j = 0;
        for (int i = 0; i < status.size(); i++) {
            if (status[i])
                inputs[j++] = inputs[i];
        }
        inputs.resize(j);
        return;
    }

    void undistort(const std::vector<cv::Point2f>& original_points, std::vector<cv::Point2f>& undistorted_points, int index, bool l_cam);

    void warpPoints(const cv::Matx33f& R, const std::vector<cv::Point2f>& in_points, std::vector<cv::Point2f>& out_points);

    bool inBorder(const cv::Point2f& pt) {
        const int border_edge = 2;
        const int x = std::round(pt.x);
        const int y = std::round(pt.y);
        return x >= border_edge && x < p_.col - border_edge && y >= border_edge && y < p_.row - border_edge;
    }

    int assignGridId(const cv::Point2f& pt) {
        int row_id = std::round(pt.y / p_.grid_height);
        int col_id = std::round(pt.x / p_.grid_width);
        int grid_id = row_id * p_.grid_col + col_id;
        return grid_id;
    }

    std::vector<std::vector<FeatureHandler::BackendFeature>> fundamentalMatrixRANSAC(
        std::vector<std::vector<Feature>>& features, 
        const sensor_msgs::ImageConstPtr& cam0,
        const sensor_msgs::ImageConstPtr& cam1);

    void publishFeatureImage(const std::vector<FeatureHandler::BackendFeature>& features, const cv::Mat& ptrLeft, const cv::Mat& ptrRight);

    std::vector<std::map<int, std::vector<Feature>>> tracked_grid_features_;

    int frame_counter_ = 0;

    bool initialized = false;

    std::vector<FeatureHandler::BackendFeature> result_;

    std::pair<uint64_t,std::vector<float>> recent_vel;

    ros::Time time_curr_frame_;
    ros::Time time_prev_frame_;

    std::vector<cv::Mat> prev_img_L_;
    std::vector<cv::Mat> prev_img_R_;

    std::vector<float> baselines;
    
    cv::Mat cov_mat_ = cv::Mat::zeros(9,9,CV_32F);

    unsigned int max_id_ = 0;
    
    ros::Publisher stereo_debug_pub_0;
    ros::Publisher ransac_features_pub;
    ros::Publisher backend_features_pub;

    std::unordered_map<int,int> feature_map_;

    int max_ransac_fid_;

    Parameters p_;
};

#endif