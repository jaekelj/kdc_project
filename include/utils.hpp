#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <Eigen/Dense>

namespace Eigen {
  typedef Matrix<float,6,1> Vector6f;
}

void quat2Rot(float x, float y, float z, float w, cv::Matx33f& rotm);

void quat2Rot(float x, float y, float z, float w, cv::Mat& rotm);

cv::Mat buildSkewSym(const cv::Mat& vec);

#endif
