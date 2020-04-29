#include <DynamicsFactor.hpp>

//------------------------------------------------------------------------------
// Inner class PreintegratedCombinedMeasurements methods
//------------------------------------------------------------------------------
void PreintegratedCombDynamicsMeasurements::setupDragMatrix(const Eigen::Matrix<double, 3, 3>& 
                                                            drag_mat) {


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

void PreintegratedCombDynamicsMeasurements::predictRotation() {

}

void PreintegratedCombDynamicsMeasurements::predictVelocity() {

}

void PreintegratedCombDynamicsMeasurements::predictPosition() {

}

void PreintegratedCombDynamicsMeasurements::integrateMeasurement() {

}



//------------------------------------------------------------------------------
// class DynamicsFactor methods
//------------------------------------------------------------------------------
void DynamicsFactor::evaluateError(const Pose3& pose_i, const Pose3& pose_j, 
                      const Vector3& vel_i, const Vector3& vec_i) {


}