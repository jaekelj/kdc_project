#ifndef DYNAMICS_FACTOR_HPP
#define DYNAMICS_FACTOR_HPP

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <cmath>
#include <math.h>

// ! BAD PRACTICE !, I know. 
using namespace gtsam;

#ifdef GTSAM_TANGENT_PREINTEGRATION
typedef TangentPreintegration PreintegrationType;
#else
typedef ManifoldPreintegration PreintegrationType;
#endif

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

      DynamicsParams() : g_vec(Vector3(0.0, 0.0, 9.81)), // ! HARD-CODING ! g vec in NED frame
                          D(Eigen::Matrix3d::Zero()),
                          dR(Eigen::Matrix3d::Identity()),
                          dtij(0.0f),
                          dtij_1(0.0f),
                          dT_nav_1(Eigen::Vector3d::Zero()),
                          dD_nav_1(Eigen::Matrix3d::Zero()),
                          Pi_g(Eigen::Vector3d::Zero()),
                          Pi_T(Eigen::Vector3d::Zero()),
                          Pi_D(Eigen::Matrix3d::Zero()),
                          Pi_Dg(Eigen::Matrix3d::Zero()),
                          Pi_DT(Eigen::Vector3d::Zero()),
                          Pi_DD(Eigen::Matrix3d::Zero()),
                          SPi_g(Eigen::Vector3d::Zero()),
                          SPi_T(Eigen::Vector3d::Zero()),
                          SPi_D(Eigen::Matrix3d::Zero()),
                          SPi_Dg(Eigen::Matrix3d::Zero()),
                          SPi_DT(Eigen::Vector3d::Zero()),
                          SPi_DD(Eigen::Matrix3d::Zero()) {}
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
    void integrateMeasurement(const Eigen::Vector3d& T_b,
                              const Eigen::Matrix<double,7,1>& imu_measurement,
                              const double& dt);
    void resetParams();    
    
    Eigen::Matrix3d predictRotation(const Eigen::Matrix3d& R_i);
    
    Eigen::Vector3d predictPosition(const Eigen::Matrix3d& R_i, const Eigen::Vector3d& v_i,
                                    const Eigen::Vector3d& v_j, const Eigen::Vector3d& p_i);

    Eigen::Vector3d predictVelocity(const Eigen::Vector3d& v_i, const Eigen::Matrix3d& R_i);
    
    Eigen::Matrix3d getSkew(const Eigen::Vector3d& x);
    Eigen::Matrix3d getExpMap(const Eigen::Vector3d& x);

}; // class PreintegratedCombDynamicsMeasurements

class DynamicsFactor : public NoiseModelFactor4<Pose3, Vector3, 
                                                Pose3, Vector3>{
    
  private:
    typedef DynamicsFactor This;
    typedef NoiseModelFactor4<Pose3, Vector3, 
                              Pose3, Vector3> Base;

    PreintegratedCombDynamicsMeasurements _PIDM_;

  public: 
    
    void evaluateError(const Pose3& pose_i, const Pose3& pose_j, 
                      const Vector3& vel_i, const Vector3& vec_i);

    virtual ~DynamicsFactor() {}


}; // class DynamicsFactor
#endif