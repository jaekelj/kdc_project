#ifndef UTILS_HPP
#define UTILS_HPP

#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <Eigen/Dense>

namespace Eigen {
  typedef Matrix<float,6,1> Vector6f;
}

namespace utils {
  float bilinearInterpolation(const cv::Mat& img, float x, float y);
  Eigen::Matrix4f seToSE(const Eigen::Vector6f& se3);
  std::string type2str(int type);
  Eigen::Matrix3f scaleIntrinsics(const Eigen::Matrix3f& K, float scale);
}

cv::Mat buildSkewSym(const cv::Mat& vec);


#endif
