#include "pixelSelection.hpp"

namespace pixelSelection
{

  std::vector<cv::Point> nonmaxSuppression(const cv::Mat& gradMag, const cv::Mat& disparity, int regionSize) {
    // right now assuming small regionSize, ignoring right and bottom of image that is left over
    cv::Mat validDisparity = disparity > 0.0;
    std::vector<cv::Point> indices;
    for (int i=0; i<gradMag.cols/regionSize; i++) {
      for (int j=0; j<gradMag.rows/regionSize; j++) {
        cv::Rect roi(i*regionSize, j*regionSize, regionSize, regionSize);
        cv::Mat submat = gradMag(roi);
        cv::Mat mask = validDisparity(roi);
        // find max gradMag
        double minVal; double maxVal;
        cv::Point minLoc; cv::Point maxLoc;
        cv::minMaxLoc( submat, &minVal, &maxVal, &minLoc, &maxLoc, mask );
        if (maxVal > 0.0) {
          indices.push_back(cv::Point(maxLoc.x+i*regionSize,maxLoc.y+j*regionSize));
        }
      }
    }
    return indices;
  }

}
