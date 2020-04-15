#include "utils.hpp"

#include <utils.hpp>

void quat2Rot(float x, float y, float z, float w, cv::Matx33f &rotm)
{
    cv::Matx33f rot_matrix(1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                           2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                           2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y);

    rotm = rot_matrix;
}

void quat2Rot(float x, float y, float z, float w, cv::Mat &rotm)
{
    cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                          2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                          2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y);

    rotm = rot_matrix;
}

cv::Mat buildSkewSym(const cv::Mat& vec){
    cv::Mat result = cv::Mat::zeros(3,3,CV_32F);
    result.at<float>(0,1) = -vec.at<float>(0,2);
    result.at<float>(0,2) = vec.at<float>(0,1);
    result.at<float>(1,2) = -vec.at<float>(0,0);
    result.at<float>(1,0) = vec.at<float>(0,2);
    result.at<float>(2,0) = -vec.at<float>(0,1);
    result.at<float>(2,1) = vec.at<float>(0,0);
    
    return result;
}