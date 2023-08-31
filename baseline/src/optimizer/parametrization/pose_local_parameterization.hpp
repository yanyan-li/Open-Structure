//
// Created by lixin04 on 19年11月22日.
//

#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

class LocalQuaternParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 4; };
  virtual int LocalSize() const { return 3; };
};

class PoseLocalEularParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 6; };
  virtual int LocalSize() const { return 6; };
};
