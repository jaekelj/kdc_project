#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

// STL includes
#include <vector>
#include <string>

// GTSAM includes
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

//OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// ROS includes
#include <ros/ros.h>

// Local includes
#include <utils.hpp>

class Parameters{
    public:
        void readConfig(std::string file_path);

        // Number of stereo camera pairs
        int num_pairs;

        // Size of image
        int row, col;

        // Camera intrinsics
        std::vector<cv::Mat> intrinsics_L;
        std::vector<cv::Mat> intrinsics_R;
        std::vector<gtsam::Vector5,Eigen::aligned_allocator<gtsam::Vector5> > intrinsics_vec_L;
        std::vector<gtsam::Vector5,Eigen::aligned_allocator<gtsam::Vector5> > intrinsics_vec_R;

        // Camera extrinsics
        std::vector<cv::Mat> R_imu_camL;
        std::vector<cv::Mat> t_imu_camL;
        std::vector<cv::Mat> R_imu_camR;
        std::vector<cv::Mat> t_imu_camR;
        std::vector<cv::Mat> R_camL_imu;
        std::vector<cv::Mat> t_camL_imu;
        std::vector<cv::Mat> R_camR_imu;
        std::vector<cv::Mat> t_camR_imu;
        std::vector<cv::Mat> T_imu_camL;
        std::vector<cv::Mat> T_camL_imu;
        std::vector<cv::Mat> R_camL_camR;
        std::vector<cv::Mat> t_camL_camR;
        std::vector<cv::Mat> t_camL_camR_hat; //Skew symmetric matrix
        std::vector<cv::Mat> left_cam_extrinsics;
        std::vector<cv::Mat> right_cam_extrinsics;
        std::vector<float> baselines;
        cv::Mat T_W_imu;

        // Epipolar geometry
        std::vector<cv::Mat> F;

        // Camera distortion params
        std::vector<cv::Mat> distortion_coeffs_L;
        std::vector<cv::Mat> distortion_coeffs_R;

        // Topic names
        std::string imu_topic;
        std::string motors_topic;
        std::string rotors_topic;
        std::vector<std::string> left_image_topics;
        std::vector<std::string> right_image_topics;
        std::string visualize_topic;

        // Optimization params
        float window_size;
        float freq;
        float max_iter;
        float abs_tol;
        float rel_tol;
        float preint_cov;
        float preint_bias_cov;
        float keyframe_spacing;
        int pub_rate;
        bool fixlag;
        bool imurate;

        // IMU Noise Params
        float sigma_a;
        float sigma_g;
        float sigma_ba;
        float sigma_bg;

        // Camera noise model param
        float sigma_pix;

        // KLT params
        int patch_size;
        int pyramid_levels;
        int klt_max_iterations;
        float klt_track_precision;
        float min_dist;


        // Feature tracking params
        int grid_row;
        int grid_col;
        int grid_height;
        int grid_width;
        int grid_min_features;
        int grid_max_features;
        int num_features;

        // RANSAC params
        float ransac_thresh;

        // Debug params
        int add_features;

    private:
        cv::Mat buildSkewSym(const cv::Mat& vec);

};

#endif
