/**
 * @file LandmarkFactors.hpp
 * @author Sudharshan Suresh (suddhu@cmu.edu)
 * @date 2018-05-08
 * @brief Factors for landmarks (corrected for refr)
 */

#ifndef STEREOLANDMARKFACTOR_H
#define STEREOLANDMARKFACTOR_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/optional/optional_io.hpp>

namespace gtsam
{
class StereoLandmarkFactor : public NoiseModelFactor2<Pose3, Point3>
{
  Pose3 mas_pose_, slv_pose_;   // transform from vehicle to master camera and vehicle to slave camera
  Point2 mas_meas_, slv_meas_; // [x,y] at each camera
  Cal3_S2 mas_cal_, slv_cal_;   // calib parameters of each camera

  #ifndef PI
  const double PI = 3.14159265358979323846;
  #endif

public:
  StereoLandmarkFactor(Key j1, Key j2, const SharedNoiseModel& model,
                  const Pose3& mas_pose, const Pose3& slv_pose,
                  const Vector5& mas_K, const Vector5& slv_K,
                  const Point2& mas_points, const Point2& slv_points):
    NoiseModelFactor2<Pose3, Point3>(model, j1, j2), // Pose3, Point3 
    mas_pose_(mas_pose), // master camera pose 
    slv_pose_(slv_pose), // slave camera pose 
    mas_meas_(mas_points), // points in mas camera 
    slv_meas_(slv_points), // points in slave camera 
    mas_cal_(mas_K(0),mas_K(1),mas_K(2),mas_K(3),mas_K(4)),   // mas calib params
    slv_cal_(slv_K(0),slv_K(1),slv_K(2),slv_K(3),slv_K(4))    // slave calib params
    {}

  Vector evaluateError(const Pose3& vehicle_pose, const Point3& point,
    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const
  {
    Matrix mas_tmp, slv_tmp;

    auto err = [&] (const Pose3& this_pose, const Point3& this_point)
    { 
      // Pose3 slv_tf = mas_pose_*(slv_pose_.inverse());
      Pose3 mas = this_pose * mas_pose_; // obtain global pose of master
      // Pose3 slv = this_pose * slv_tf; // obtain global pose of slave
      Pose3 slv = this_pose * slv_pose_; // obtain global pose of slave

      // std::clog<<"this_pose: "<<this_pose<<std::endl;
      // std::clog<<"mPo : "<<mas<<std::endl;
      // std::clog<<"sPo : "<<slv<<std::endl;
      // std::clog<<"mas_pose_ : "<<mas_pose_<<std::endl;
      // std::clog<<"slv_tf : "<<slv_tf<<std::endl;

      // defines camera objects
      SimpleCamera mas_c(mas, mas_cal_);
      SimpleCamera slv_c(slv, slv_cal_);

      // iterative method to obtain virtual point
      Point3 Pm, Ps; 
      Point2 mas_pt, slv_pt; 

      try {
      // without correction
      mas_pt = mas_c.project2(this_point, mas_tmp);
      slv_pt = slv_c.project2(this_point, slv_tmp);
      }
      catch( gtsam::CheiralityException &e)
      {
        mas_pt = gtsam::Point2(-100,-100);
        slv_pt = gtsam::Point2(-100,-100);
      }

      // std::clog<<"mpx : "<<mas_pt<<std::endl;
      // std::clog<<"spx : "<<slv_pt<<std::endl;

      // std::clog<<"mmeas : "<<mas_meas_<<std::endl;
      // std::clog<<"smeas : "<<slv_meas_<<std::endl;
      Vector4 pred, meas;
      pred << mas_pt.x(), mas_pt.y(), slv_pt.x(), slv_pt.y();
      meas << mas_meas_.x(), mas_meas_.y(), slv_meas_.x(), slv_meas_.y();

      // std::clog<<"p: "<<pred.transpose()<<" m: "<<meas.transpose()<<"\n";
      return Vector(pred - meas);
    };

    // numerical derivative method 
    if (H1) (*H1) = numericalDerivative21<Vector, Pose3, Point3>(err, vehicle_pose, point);
    if (H2) (*H2) = numericalDerivative22<Vector, Pose3, Point3>(err, vehicle_pose, point);

    return err(vehicle_pose, point);
  }
};

} // namespace gtsam

#endif // STEREOLANDMARKFACTOR_H