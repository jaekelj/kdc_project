#ifndef WEIGHT_HPP
#define WEIGHT_HPP

#include "opencv2/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>

namespace weight {

  const float Huber_constant = 1.345;
  const float Tukey_constant = 4.6851;
  const float Tukey_constant2 = Tukey_constant*Tukey_constant;

  const int sigma_I = 4;
  const int sigma_I2 = sigma_I*sigma_I;
  const int two_sigma_I2 = 2*sigma_I2;

  float stereo_weight(float residual, float gx, float gy, float fx, float fy, const Eigen::Vector3f& input_pt, const Eigen::Matrix3f& R, const Eigen::Vector3f& ref_pt, float disparity, float dx);

}

#endif
