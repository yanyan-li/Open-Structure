//
// Created by lixin04 on 19年11月22日.
//

#include "pose_local_parameterization.hpp"

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
    const Eigen::MatrixBase<Derived>& theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

bool LocalQuaternParameterization::Plus(const double* x,
                                        const double* delta,
                                        double* x_plus_delta) const {
  Eigen::Map<const Eigen::Quaterniond> _q(x);

  Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));

  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

  q = (_q * dq).normalized();
  return true;
}

bool LocalQuaternParameterization::ComputeJacobian(const double* x,
                                                   double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
  j.topRows<3>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalParameterization::Plus(const double* x,
                                     const double* delta,
                                     double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);  // double 指针 转为eigen数组
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double* x,
                                                double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}

bool PoseLocalEularParameterization::Plus(const double* x,
                                          const double* delta,
                                          double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Vector3d> _theta(x + 3);

  double s1 = sin(_theta[0]);
  double c1 = cos(_theta[0]);
  double s2 = sin(_theta[1]);
  double c2 = cos(_theta[1]);
  double s3 = sin(_theta[2]);
  double c3 = cos(_theta[2]);
  Eigen::Matrix3d _R;
  _R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, c2 * s3,
      s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, -s2, s1 * c2, c1 * c2;

  Eigen::Map<const Eigen::Vector3d> dp(delta);
  Eigen::Map<const Eigen::Vector3d> delta_theta(delta + 3);
  Eigen::Matrix3d Rz;
  Rz << cos(delta_theta(2)), -sin(delta_theta(2)), 0, sin(delta_theta(2)),
      cos(delta_theta(2)), 0, 0, 0, 1;

  Eigen::Matrix3d Ry;
  Ry << cos(delta_theta(1)), 0., sin(delta_theta(1)), 0., 1., 0.,
      -sin(delta_theta(1)), 0., cos(delta_theta(1));

  Eigen::Matrix3d Rx;
  Rx << 1., 0., 0., 0., cos(delta_theta(0)), -sin(delta_theta(0)), 0.,
      sin(delta_theta(0)), cos(delta_theta(0));
  _R = _R * Rx * Ry * Rz;
  //    _R = _R * Rz * Ry * Rx;

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);  // double 指针 转为eigen数组
  Eigen::Map<Eigen::Vector3d> theta(x_plus_delta + 3);

  p = _p + dp;
  Eigen::Vector3d u1 = _R.col(0);
  Eigen::Vector3d u2 = _R.col(1);
  Eigen::Vector3d u3 = _R.col(2);
  theta[0] = atan2(u2(2), u3(2));
  theta[1] = asin(-u1(2));
  theta[2] = atan2(u1(1), u1(0));

  return true;
}
bool PoseLocalEularParameterization::ComputeJacobian(const double* x,
                                                     double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacobian);
  j.setIdentity();

  return true;
}