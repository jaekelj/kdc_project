#include "utils.hpp"

namespace utils {

  float bilinearInterpolation(const cv::Mat& img, float x, float y)
  {
    int x0 = (int) x;
    int y0 = (int) y;
    // int x0 = cv::borderInterpolate(x_int,   img.cols, cv::BORDER_REFLECT_101);
    // int x1 = cv::borderInterpolate(x_int+1, img.cols, cv::BORDER_REFLECT_101);
    // int y0 = cv::borderInterpolate(y_int,   img.rows, cv::BORDER_REFLECT_101);
    // int y1 = cv::borderInterpolate(y_int+1, img.rows, cv::BORDER_REFLECT_101);

    float a = x - x0;
    float c = y - y0;

    float val = (img.at<uchar>(y0,x0)*(1.0-a) + img.at<uchar>(y0,x0+1)*a)*(1.0-c)
              + (img.at<uchar>(y0+1,x0)*(1.0-a) + img.at<uchar>(y0+1,x0+1)*a)*c;

    return val;
  }

  Eigen::Matrix4f seToSE(const Eigen::Vector6f& se3) {
    Eigen::Vector3f w = se3.head<3>();
    Eigen::Vector3f v = se3.tail<3>();
    float phi = w.norm();
    float phi_inv = 1/phi;
    float phi_inv2 = phi_inv*phi_inv;
    float phi_inv3 = phi_inv2*phi_inv;
    float sin_phi = std::sin(phi);
    float one_minus_cos_phi = 1-std::cos(phi);
    Eigen::Matrix3f wx;
    wx << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    Eigen::Matrix3f wx2 = wx*wx;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + phi_inv*sin_phi*wx + phi_inv2*one_minus_cos_phi*wx2;
    Eigen::Matrix4f T = Eigen::Matrix4f::Zero();
    T.topLeftCorner(3,3) = R;
    Eigen::Matrix3f V = Eigen::Matrix3f::Identity();
    if (phi > 0.001) {
      V += phi_inv2*one_minus_cos_phi*wx + phi_inv3*(phi-sin_phi)*wx2;
    }
    T.topRightCorner(3,1) = V*v;
    T(3,3) = 1.0;
    return T;
  }

  std::string type2str(int type) {
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
    }
    r += "C";
    r += (chans+'0');
    return r;
  }

  Eigen::Matrix3f scaleIntrinsics(const Eigen::Matrix3f& K, float scale) {
    Eigen::Matrix3f K_new = K;
    K_new(0,0) *= scale;
    K_new(1,1) *= scale;
    K_new(0,2) *= scale;
    K_new(1,2) *= scale;
    return K_new;
  }

} // end namespace utils
