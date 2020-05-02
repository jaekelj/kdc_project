#include "imageProcessing.hpp"

namespace imageProcessing {

  void getGradients(const cv::Mat& gray, cv::Mat& gradX, cv::Mat& gradY, cv::Mat& gradMag) {
    int ddepth = CV_32F;
    float scaleScharr = 1.0/32.0;
    //sscaleScharrr = 100.0;
    cv::Scharr(gray, gradX, ddepth, 1, 0, scaleScharr);
    cv::Scharr(gray, gradY, ddepth, 0, 1, scaleScharr);
    gradMag = gradX.mul(gradX) + gradY.mul(gradY);
  }

  void getGradientsNoMag(const cv::Mat& gray, cv::Mat& gradX, cv::Mat& gradY) {
    int ddepth = CV_32F;
    float scaleScharr = 1.0/32.0;
    cv::Scharr(gray, gradX, ddepth, 1, 0, scaleScharr);
    cv::Scharr(gray, gradY, ddepth, 0, 1, scaleScharr);
  }

  std::vector<cv::Mat> createImagePyramid(const cv::Mat& gray, int numLevels, int firstLevel) {

    std::vector<cv::Mat> imgPyr;
    cv::Mat currLevel = gray.clone();
    for (int i=0; i<firstLevel; i++) {
      pyrDown(currLevel, currLevel);
    }
    for (int i=0; i<numLevels; i++) {
      imgPyr.push_back(currLevel);
      pyrDown(currLevel, currLevel);
    }
    std::reverse(imgPyr.begin(), imgPyr.end());
    return imgPyr;
  }

  void createImageAndDisparityPyramid(const cv::Mat& gray, const cv::Mat& disparity, int numLevels, int firstLevel,
    std::vector<cv::Mat>& grayPyr, std::vector<cv::Mat>& disparityPyr) {

    grayPyr.clear();
    disparityPyr.clear();
    cv::Mat currGray = gray.clone();
    cv::Mat currDisparity;
    for (int i=0; i<firstLevel; i++) {
      pyrDown(currGray, currGray);
    }
    for (int i=0; i<numLevels; i++) {
      grayPyr.push_back(currGray);
      cv::resize(disparity,currDisparity,currGray.size(),0.0,0.0,cv::INTER_NEAREST);
      disparityPyr.push_back(currDisparity);
      pyrDown(currGray, currGray);
    }
    std::reverse(grayPyr.begin(), grayPyr.end());
    std::reverse(disparityPyr.begin(), disparityPyr.end());
  }

} // end namespace imageProcessing
