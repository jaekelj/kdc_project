#include <DynamicsFactor.hpp>

//------------------------------------------------------------------------------
// Inner class PreintegratedCombinedMeasurements methods
//------------------------------------------------------------------------------
void PreintegratedCombDynamicsMeasurements::setupDragMatrix(const Eigen::Matrix<double, 3, 3>& 
                                                            drag_mat) {

  this->dyn_params.D = drag_mat;
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// integrates one measurement of mass-normalized thrust T_b, and un-biased gyro
// imu_measurement for time interval dt
void PreintegratedCombDynamicsMeasurements::integrateMeasurement(const Eigen::Vector3d& T_b,
                                                                const Eigen::Matrix<double,7,1>& imu_measurement,
                                                                const double& dt) {

  // taking out the gyro values from the imu measurement vector                                                                
  Eigen::Vector3d gyr(imu_measurement(4), imu_measurement(5), imu_measurement(6));
  
  // deltas in nav state
  Eigen::Matrix<double, 3, 1> dT_nav(this->dyn_params.dR * T_b * dt); 
  Eigen::Matrix<double, 3, 3> dD_nav(this->dyn_params.dR 
                                    * this->dyn_params.D 
                                    * this->dyn_params.dR.transpose()
                                    * dt);

  // integrating time
  this->dyn_params.dtij += dt;

  // integrating rotation
  this->dyn_params.dR *= getExpMap(gyr * dt);

  // integrating position terms
  double dtbar = (dt + this->dyn_params.dt_1) / 2;
  this->dyn_params.SPi_g += this->dyn_params.Pi_g * dtbar;
  this->dyn_params.SPi_T += this->dyn_params.Pi_T * dtbar;
  this->dyn_params.SPi_D += this->dyn_params.Pi_D * dtbar;
  this->dyn_params.SPi_Dg += this->dyn_params.Pi_Dg * dtbar;
  this->dyn_params.SPi_DT += this->dyn_params.Pi_DT * dtbar;
  this->dyn_params.SPi_DD += this->dyn_params.Pi_DD * dtbar; 

  // integrating velocity terms
  this->dyn_params.Pi_Dg = this->dyn_params.dD_nav_1 * this->dyn_params.dtij_1;
  this->dyn_params.Pi_DT = this->dyn_params.dD_nav_1 * this->dyn_params.Pi_T;
  this->dyn_params.Pi_DD = this->dyn_params.dD_nav_1 * this->dyn_params.Pi_D;
  this->dyn_params.Pi_g += this->dyn_params.g_vec * dt; 
  this->dyn_params.Pi_T += dT_nav;
  this->dyn_params.Pi_D += dD_nav;
  
  // updating values for the next iteration
  this->dyn_params.dt_1 = dt;
  this->dyn_params.dtij_1 = this->dyn_params.dtij;
  this->dyn_params.dT_nav_1 = dT_nav;
  this->dyn_params.dD_nav_1 = dD_nav;

}

//------------------------------------------------------------------------------
// Predicts rotation given rotation R_i at state i
// (given all preintegrated measurements so far)
Eigen::Matrix3d PreintegratedCombDynamicsMeasurements::predictRotation( \
                                                        const Eigen::Matrix3d& R_i) {
  Eigen::Matrix3d R_j = R_i * this->dyn_params.dR;
  return R_j;
}

//------------------------------------------------------------------------------
// Predicts velocity given rotation R_i and velocity v_i at state i
// (given all preintegrated measurements so far)
Eigen::Vector3d PreintegratedCombDynamicsMeasurements::predictVelocity( \
                                                        const Eigen::Vector3d& v_i,
                                                        const Eigen::Matrix3d& R_i) {

  Eigen::Vector3d v_j = Eigen::Vector3d::Zero();
  v_j = v_i  
        + this->dyn_params.Pi_g 
        + R_i * this->dyn_params.Pi_T
        - R_i * this->dyn_params.Pi_D * R_i.transpose() * v_i
        - R_i * this->dyn_params.Pi_Dg * R_i.transpose() * this->dyn_params.g_vec
        - R_i * this->dyn_params.Pi_DT
        + R_i * this->dyn_params.Pi_DD * R_i.transpose() * v_i;

  return v_j;
}

//------------------------------------------------------------------------------
// Predicts position given all the preintegrated measurements so far
// Given --> R_i, v_i, v_j, p_i
Eigen::Vector3d PreintegratedCombDynamicsMeasurements::predictPosition(
                                    const Eigen::Matrix3d& R_i, const Eigen::Vector3d& v_i,
                                    const Eigen::Vector3d& v_j, const Eigen::Vector3d& p_i) {
  
  Eigen::Vector3d p_j = Eigen::Vector3d::Zero();
  p_j = p_i
      + this->dyn_params.dtij * v_i
      + this->dyn_params.dt_1 * (v_j - v_i)/2
      + this->dyn_params.SPi_g
      + R_i * this->dyn_params.SPi_T
      - R_i * this->dyn_params.SPi_D * R_i.transpose() * v_i
      - R_i * this->dyn_params.SPi_Dg * R_i.transpose() * this->dyn_params.g_vec
      - R_i * this->dyn_params.SPi_DT
      + R_i * this->dyn_params.SPi_DD * R_i.transpose() * v_i;
  
  return p_j;
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

//------------------------------------------------------------------------------
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
    
    // ! RE-CHECK ! --> Unsure about the computation of the exponential map
    
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