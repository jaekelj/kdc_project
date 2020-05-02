#include <MultiDvo.hpp>

MultiDvo::MultiDvo(int num_scales, int first_scale) {
  num_scales_ = num_scales;
  first_scale_ = first_scale;

  T_relative_ = Eigen::Matrix4f::Identity();
  T_init_ = Eigen::Matrix4f::Identity();
}

void MultiDvo::track(const std::vector<cv::Mat>& inputs) {

  for (int i=0; i<dvo_.size(); i++) {
    dvo_[i]->setInput(inputs[i]);
  }

  Eigen::Matrix6f A;
  Eigen::Matrix4f T = T_init_;  // TO DO: MOTION MODEL
  for (int level = 0; level < num_scales_; level++) {
    Eigen::Vector6f deltat_prev = Eigen::Vector6f::Zero();
    float cost_sum_prev = 0;
    for (int iter=0; iter<max_iter_; iter++) {
      A = Eigen::Matrix6f::Zero();
      Eigen::Vector6f b = Eigen::Vector6f::Zero();
      float cost_sum = 0;
      for (int i=0; i<dvo_.size(); i++) {
        dvo_[i]->constructSystem(T, level);
        A += dvo_[i]->A();
        b += dvo_[i]->b();
        cost_sum += dvo_[i]->cost_sum();
      }

      Eigen::Vector6f deltat;
      T = solveSystem(T, A, b, deltat);

      // convergence checks
      if ( (deltat - deltat_prev).norm() < change_thresh_)
        break;
      else if (deltat.norm() < norm_thresh_)
        break;
      else if (std::fabs(cost_sum-cost_sum_prev) < cost_thresh_)
        break;

      deltat_prev = deltat;
      cost_sum_prev = cost_sum;

    }
  }
  T_relative_ = T;
  information_ = A;
}

Eigen::Matrix4f MultiDvo::solveSystem(const Eigen::Matrix4f& T, const Eigen::Matrix6f& A, const Eigen::Vector6f& b,
  Eigen::Vector6f& deltat ) {

  deltat = A.ldlt().solve(b);
  Eigen::Matrix4f deltaT = utils::seToSE(-deltat);
  Eigen::Matrix4f T_potential = T*deltaT;
  return T_potential;
}
