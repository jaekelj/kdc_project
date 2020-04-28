#ifndef DYNAMICS_FACTOR_HPP
#define DYNAMICS_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>

class PreintegratedCombDynamicsMeasurements : public PreintegrationType {

  public:

    struct DynamicsParams {
      Eigen::Matrix<double, 3, 1> g_vec; // gravity vector
      Eigen::Matrix<double, 3, 3> D; // drag matrix
      Eigen::Matrix<double, 3, 3> dR; // rotation
      
      double dtij; // delta t from i to j
      double dtij_1; // last dtij
      double dt_1; // last dt
      
      Eigen::Matrix<double, 3, 1> dT_nav_1; // last dT_nav
      Eigen::Matrix<double, 3, 3> dD_nav_1; // last dD_nav

      Eigen::Matrix<double, 3, 1> Pi_g;
      Eigen::Matrix<double, 3, 1> Pi_T;
      Eigen::Matrix<double, 3, 3> Pi_D;
      Eigen::Matrix<double, 3, 3> Pi_Dg;
      Eigen::Matrix<double, 3, 1> Pi_DT;
      Eigen::Matrix<double, 3, 3> Pi_DD;

      Eigen::Matrix<double, 3, 1> SPi_g;
      Eigen::Matrix<double, 3, 1> SPi_T;
      Eigen::Matrix<double, 3, 3> SPi_D;
      Eigen::Matrix<double, 3, 3> SPi_Dg;
      Eigen::Matrix<double, 3, 1> SPi_DT;
      Eigen::Matrix<double, 3, 3> SPi_DD;

      DynamicsParams() : g_vec {Vector3{0, 0, 9.81}},
                          D {Z_3x3},
                          dR {I_3x3},
                          dtij {0.0f},
                          dtij_1 {0.0f},
                          dT_nav_1 {Z_3x1},
                          dD_nav_1 {Z_3x3},
                          Pi_g {Z_3x1},
                          Pi_T {Z_3x1},
                          Pi_D {Z_3x3},
                          Pi_Dg {Z_3x3},
                          Pi_DT {Z_3x1},
                          Pi_DD {Z_3x3},
                          SPi_g {Z_3x1},
                          SPi_T {Z_3x1},
                          SPi_D {Z_3x3},
                          SPi_Dg {Z_3x3},
                          SPi_DT {Z_3x1},
                          SPi_DD {Z_3x3} {}
    };

    DynamicsParams dyn_params;

  protected: 
    /* Ideally there should be a covariance matrix of preintegrated measurements here
     *
     */

    friend class DynamicsFactor;

  public:
    PreintegratedCombDynamicsMeasurements() : dyn_params() {}
    /**
     * Add a single dynamics measurement to the preintegration
     * 
     */
    void setupDragMatrix(const Eigen::Matrix<double, 3, 3>& drag_mat);
    void integrateMeasurement();
    void resetParams();    
    void predictRotation();
    void predictPosition();
    void predictVelocity();

}; // class PreintegratedCombDynamicsMeasurements

class DynamicsFactor : public NoiseModelFactor4<Pose3, Vector3, Pose3, Vector3>{
    
  private:
    typedef DynamicsFactor This;
    typedef NoiseModelFactor4<Pose3, Vector3, Pose3, Vector3> Base;

    PreintegratedCombDynamicsMeasurements _PIDM_;

  public: 
    
    void evaluateError(const Pose3& pose_i, const Pose3& pose_j, 
                      const Vector3& vel_i, const Vector3& vec_i);

    virtual ~DynamicsFactor() {}


}; // class DynamicsFactor
#endif