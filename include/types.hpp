#ifndef TYPES_HPP
#define TYPES_HPP

namespace Eigen {
  typedef Matrix<float,6,1> Vector6f;
  typedef Matrix<float,6,6> Matrix6f;
  typedef Matrix<float,2,Dynamic> Matrix2Xf;
  typedef Matrix<float,3,Dynamic> Matrix3Xf;
  typedef Matrix<float,4,Dynamic> Matrix4Xf;
  typedef Matrix<float,Dynamic,6> MatrixX6f;
}

#endif
