#include "BlockMatching.hpp"
#include <iostream>

BlockMatching::BlockMatching(cv::Size s) {
  minDisparity=0;
  blockSize=9;
  preFilterCap=31;
  //numDisparities=((s.width/20) + 15) & -16;
  numDisparities = ((s.width/10) + 15) & -16;
  //std::cout << numDisparities << std::endl;
  textureThreshold=10;
  uniquenessRatio=15;
  speckleWindowSize=100;
  speckleRange=32;
  disp12MaxDiff=1;

  bm = cv::StereoBM::create();
  bm->setMinDisparity(minDisparity);
  bm->setBlockSize(blockSize);
  bm->setPreFilterCap(preFilterCap);
  bm->setNumDisparities(numDisparities);
  bm->setTextureThreshold(textureThreshold);
  bm->setUniquenessRatio(uniquenessRatio);
  bm->setSpeckleWindowSize(speckleWindowSize);
  bm->setSpeckleRange(speckleRange);
  bm->setDisp12MaxDiff(disp12MaxDiff);
}

cv::Mat BlockMatching::compute(const cv::Mat& left, const cv::Mat& right) {
  cv::Mat dispShort;
  bm->compute(left, right, dispShort);
  cv::Mat disp;
  dispShort.convertTo(disp, CV_32F, 1.0/16.0);
  return disp;
}
