#ifndef MULTIDVO_HPP
#define MULTIDVO_HPP

#include "opencv2/core.hpp"

#include "utils.hpp"
#include "imageProcessing.hpp"
#include "pixelSelection.hpp"
#include "weight.hpp"
#include "BlockMatching.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include "types.hpp"
#include "Dvo.hpp"

class MultiDvo {
  private:

    std::vector<std::shared_ptr<Dvo> > dvo_;

    Eigen::Matrix4f T_relative_;
    Eigen::Matrix6f information_;

    // motion model initializaiton
    Eigen::Matrix4f T_init_;

    // pyramid parameters
    int first_scale_;
    int num_scales_;

    // convergence criteria
    int max_iter_;
    float gradient_thresh_;
    float norm_thresh_;
    float change_thresh_;
    float cost_thresh_;

  public:
    MultiDvo(int num_scales, int first_scale);
    ~MultiDvo() {;};

    void track(const std::vector<cv::Mat>& inputs);

    Eigen::Matrix4f solveSystem(const Eigen::Matrix4f& T, const Eigen::Matrix6f& A, const Eigen::Vector6f& b,
      Eigen::Vector6f& deltat );

    // add dvo
    void addDvo(std::shared_ptr<Dvo> dvo) {dvo_.push_back(dvo);}

    Eigen::Matrix4f getRelativePose() {return T_relative_;}
    Eigen::Matrix6f getInformation() {return information_;}
    Eigen::Matrix6f getCovariance() {return information_.inverse();}

    void setInitTransform(const Eigen::Matrix4f& T) {T_init_ = T;}

    // convergence criteria
    void setNumMaxIter(int max_iter) {max_iter_ = max_iter;}
    void setGradientThresh(float gradient_thresh) {gradient_thresh_ = gradient_thresh;}
    void setNormThresh(float norm_thresh) {norm_thresh_ = norm_thresh;}
    void setChangeThresh(float change_thresh) {change_thresh_ = change_thresh;}
    void setCostThresh(float cost_thresh) {cost_thresh_ = cost_thresh;}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif
