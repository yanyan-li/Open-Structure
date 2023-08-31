/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-02-24 14:40:08
 * @LastEditTime: 2023-02-25 15:11:19
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /VENOM/src/optimizer/parametrization/line_parameterization.hpp
 */
#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

class LineOrthParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 4; };
  virtual int LocalSize() const { return 4; };
};


class ParaLineOrthParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 2; };
  virtual int LocalSize() const { return 2; };
};
