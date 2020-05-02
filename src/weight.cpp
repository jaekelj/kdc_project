#include "weight.hpp"

namespace weight {
  inline float huberWeight(float x) {
  	if (std::fabs(x) <= Huber_constant) {
  		return 1.0;
    }
  	else {
  		return Huber_constant/std::fabs(x);
    }
  }

  inline float tukeyWeight(float x) {
  	if (std::fabs(x) <= Tukey_constant) {
      float y = 1-x*x/Tukey_constant2;
  		return y*y;
    }
  	else {
  		return 0;
    }
  }

  float stereo_weight(float residual, float gx, float gy, float fx, float fy, const Eigen::Vector3f& input_pt, const Eigen::Matrix3f& R, const Eigen::Vector3f& ref_pt, float disparity, float dx) {

    // std::cout << residual << " " << gx << " " << gy << " " << fx << " " << fy << std::endl;
    // std::cout << input_pt.transpose() << std::endl;
    // std::cout << R << std::endl;
    // std::cout << ref_pt.transpose() << std::endl;
    // std::cout << disparity << " " << dx << std::endl;
    if (dx == 0) {
      return 0;
    }

    float Fx = gx*fx;
    float Fy = gy*fy;
    float z_input = input_pt(2);
    float z_input_inv = 1/z_input;
    Eigen::RowVector3f lhs( Fx, Fy, z_input_inv*(Fx*input_pt(0) + Fy*input_pt(1)) );
    lhs *= z_input_inv;
    float drdd = lhs*R*ref_pt;
    drdd /= disparity;
    float drdd2 = drdd*drdd;
    float sigma2_dj = two_sigma_I2/(dx*dx);
    float sigma2_rj = two_sigma_I2 + drdd2*sigma2_dj;
    float sigma_rj = std::sqrt(sigma2_rj);
    // std::cout << drdd2 << " " << sigma2_dj << " " << sigma2_rj << std::endl;
    return huberWeight(residual/sigma_rj)/sigma2_rj;
  }


}
