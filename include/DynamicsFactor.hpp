#ifndef DYNAMICS_FACTOR_HPP
#define DYNAMICS_FACTOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>

class DynamicsFactor : public NoiseModelFactor4<Pose3, Vector3, Pose3, Vector3>{
    Vector evaluateError(const Pose3& pose_i, const Pose3& pose_j, const Vector3& vel_i, const Vector3& vec_i){
        // TODO: Implement this function
        Vector a;
        return a;
    }

    // private:
    //     PreintegratedDynamicsMeasurements _PIDM_;
};

#endif