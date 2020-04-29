#include <DynamicsFactor.hpp>

//------------------------------------------------------------------------------
// Inner class PreintegratedCombinedMeasurements methods
//------------------------------------------------------------------------------
void PreintegratedCombDynamicsMeasurements::setupDragMatrix(const Eigen::Matrix<double, 3, 3>& 
                                                            drag_mat) {

  this->dyn_params.D = drag_mat;
}

void PreintegratedCombDynamicsMeasurements::resetParams() {
  
  dyn_params.D.setZero();
  dyn_params.dR.setIdentity();
  dyn_params.dtij = 0.0f;
  dyn_params.dtij_1 = 0.0f;
  dyn_params.dT_nav_1.setZero();
  dyn_params.dD_nav_1.setZero();
  dyn_params.Pi_g.setZero();
  dyn_params.Pi_T.setZero();
  dyn_params.Pi_D.setZero();
  dyn_params.Pi_Dg.setZero();
  dyn_params.Pi_DT.setZero();
  dyn_params.Pi_DD.setZero();
  dyn_params.SPi_g.setZero();
  dyn_params.SPi_T.setZero();
  dyn_params.SPi_D.setZero();
  dyn_params.SPi_Dg.setZero();
  dyn_params.SPi_DT.setZero();
  dyn_params.SPi_DD.setZero();
}

// integrates one measurement of mass-normalized thrust T_b, and un-biased gyro
// gyr for time interval dt
void PreintegratedCombDynamicsMeasurements::integrateMeasurement() {

}

// Predicts rotation given rotation R_i at state i
// (given all preintegrated measurements so far)
Eigen::Matrix3d PreintegratedCombDynamicsMeasurements::predictRotation( \
                                                        Eigen::Matrix3d& R_i) {
  Eigen::Matrix3d R_j = R_i * this->dyn_params.dR;
  return R_j;
}

// Predicts velocity given rotation R_i and velocity v_i at state i
// (given all preintegrated measurements so far)
Eigen::Vector3d PreintegratedCombDynamicsMeasurements::predictVelocity( \
                                                        Eigen::Vector3d& v_i,
                                                        Eigen::Matrix3d& R_i) {

  Eigen::Vector3d v_j = Eigen::Vector3d::Zero();


}


// Predicts position given all the preintegrated measurements so far
// Given --> R_i, v_i, v_j, p_i
Eigen::Vector3d PreintegratedCombDynamicsMeasurements::predictPosition(Eigen::Matrix3d& R_i, Eigen::Vector3d& v_i,
                                    Eigen::Vector3d& v_j, Eigen::Vector3d& p_i) {

}

// compute the skew symmetric matrix corresponding to a vector
Eigen::Matrix3d PreintegratedCombDynamicsMeasurements::getSkew(
                                                    const Eigen::Vector3d& x) {

  Eigen::Matrix3d skew_x;
  skew_x << 0.0, -x(2), x(1),
            x(2), 0, -x(0),
            -x(1), x(0), 0;
  return skew_x;          
}

// compute the exponential map
Eigen::Matrix3d PreintegratedCombDynamicsMeasurements::getExpMap(
                                                const Eigen::Vector3d& x) {

  Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
  Eigen::Matrix3d exp_x = Eigen::Matrix3d::Identity();
  if (x.isApprox(zero_vec)) { 
    return exp_x;
  }
  else { 
    double norm_x = x.norm();
    Eigen::Matrix3d skew_x = getSkew(x);
    
    // TO-DO --> Unsure about the computation of this exponential
    
    // option-1
    // exp_x = x.exp(); // this requires a square matrix

    // option-2
    exp_x += sin(norm_x)/norm_x * skew_x + \
            (1 - cos(norm_x))/pow(norm_x, 2) * (skew_x*skew_x);
  }
}


//------------------------------------------------------------------------------
// class DynamicsFactor methods
//------------------------------------------------------------------------------
void DynamicsFactor::evaluateError(const Pose3& pose_i, const Pose3& pose_j, 
                      const Vector3& vel_i, const Vector3& vec_i) {


}