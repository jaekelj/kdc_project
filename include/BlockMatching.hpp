#ifndef BLOCKMATCHING_HPP
#define BLOCKMATCHING_HPP

#include "opencv2/calib3d/calib3d.hpp"

class BlockMatching {
  private:
    int preFilterCap;
    int blockSize;
    int minDisparity;
    int numDisparities;
    int textureThreshold;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    cv::Ptr<cv::StereoBM> bm;
  public:
    BlockMatching(cv::Size s);
    cv::Mat compute(const cv::Mat& left, const cv::Mat& right);
    int getNumDisparities() {return numDisparities;}
};

#endif
