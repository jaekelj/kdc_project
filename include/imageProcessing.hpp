#ifndef IMAGEPROCESSING_HPP
#define IMAGEPROCESSING_HPP

#include "opencv2/imgproc.hpp"

namespace imageProcessing {

  void getGradients(const cv::Mat& gray, cv::Mat& gradX, cv::Mat& gradY, cv::Mat& gradMag);

  void getGradientsNoMag(const cv::Mat& gray, cv::Mat& gradX, cv::Mat& gradY);

  std::vector<cv::Mat> createImagePyramid(const cv::Mat& gray, int numLevels, int firstLevel);

  void createImageAndDisparityPyramid(const cv::Mat& gray, const cv::Mat& disparity, int numLevels, int firstLevel,
    std::vector<cv::Mat>& grayPyr, std::vector<cv::Mat>& disparityPyr);

}

#endif
