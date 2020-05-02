#ifndef DVO_HPP
#define DVO_HPP

#include <random>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "utils.hpp"
#include "imageProcessing.hpp"
#include "pixelSelection.hpp"
#include "weight.hpp"
#include "BlockMatching.hpp"
#include "types.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "happly.h"

#define _OPENMP
class Dvo {
  private:

    std::vector<Eigen::MatrixX6f> J_;
    std::vector< std::vector<Eigen::Matrix6f> > H_;
    std::vector<Eigen::VectorXf> refVal_;
    std::vector<Eigen::Matrix4Xf> refPoints_;

    std::vector<cv::Mat> input_pyr_;

    // body to camera transformation
    Eigen::Matrix4f T_c_b_;
    Eigen::Matrix3f R_c_b_;
    Eigen::Matrix4f T_b_c_;

    // camera parameters
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > K_;
    std::vector<float> baseline_;

    // rectification
    cv::Mat map0x_, map0y_;
    cv::Mat map1x_, map1y_;
    cv::Mat R0_;

    // stereo matching
    std::shared_ptr<BlockMatching> bm_;

    // topic id
    std::string cam0_topic_;
    std::string cam1_topic_;

    // linear system
    Eigen::Matrix6f A_;
    Eigen::Vector6f b_;
    float cost_sum_;

    // pyramid parameters
    int first_scale_;
    int num_scales_;

    // pixel selection
    int selection_region_size_;
    int pixel_selection_scale_;

    void constructReference(const cv::Mat& gray, const cv::Mat& disparity, int scale);

  public:
    Dvo(std::string config_file, int num_scales, int first_scale);
    ~Dvo() {;};

    void setReference(const cv::Mat& left, const cv::Mat& right);

    void constructSystem(const Eigen::Matrix4f& T, int scale);

    void readParameters(std::string config_file, int num_scales, int first_scale);

    void setInput(const cv::Mat& input);

    void setSelectionRegionSize(int selection_region_size) {selection_region_size_=selection_region_size;}
    void setPixelSelectionScale(int scale) {pixel_selection_scale_ = scale;}

    // linear system access
    Eigen::Matrix6f A() {return A_;}
    Eigen::Vector6f b() {return b_;}
    float cost_sum() {return cost_sum_;}

    // topic access
    std::string cam0Topic() {return cam0_topic_;}
    std::string cam1Topic() {return cam1_topic_;}

    /** Visualization utilities **/
    const float kMinDepth = 0.1f;
    const float kMaxDepth = 30.0f;
    cv::Mat visualizeDepth(
        cv::Mat &disparity, int scale = 0);
    cv::Mat visualizeDepthOnReference(
        cv::Mat &left, cv::Mat &disparity, int scale = 0);
    bool writeLocalMapToPly(
        const std::string &filename,
        cv::Mat &left, cv::Mat &disparity, int scale = 0);

    const float kDepthThreshold = 10.0f;
    std::vector<std::array<double, 3>> vertices_;
    std::vector<std::array<double, 3>> vertex_colors_;
    std::random_device rd_;
    void appendToGlobalMap(cv::Mat &left, cv::Mat &disparity,
                           const Eigen::Matrix4f &T,
                           float sample_rate = 0.1f,
                           int scale = 2);
    bool writeGlobalMapToPly(const std::string &filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif
