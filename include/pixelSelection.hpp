#ifndef PIXELSELECTION_HPP
#define PIXELSELECTION_HPP

#include "opencv2/imgproc.hpp"
#include <iostream>

namespace pixelSelection {

  std::vector<cv::Point> nonmaxSuppression(const cv::Mat& gradMag, const cv::Mat& disparity, int regionSize);

}

#endif
