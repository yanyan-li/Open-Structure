/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-03-05 00:39:11
 * @LastEditTime: 2023-08-04 18:41:46
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /JointFactorGraph/src/optimizer/factor/LineProjectionFactor.hpp
 */
#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>


typedef Eigen::Matrix<double, 6, 1> Vector6d;
class PointProjectionFactor : public ceres::SizedCostFunction<2, 7, 3>
{
  public:
    PointProjectionFactor(const Eigen::Vector2d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector2d obs_i;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class lineProjectionFactor : public ceres::SizedCostFunction<2, 7, 4>
{
  public:
    lineProjectionFactor(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class ParaLineProjectionFactor : public ceres::SizedCostFunction<2, 7, 2, 2>
{
  public:
    ParaLineProjectionFactor(const Eigen::Vector4d &_pts_i, const Vector6d& _PLK0, const Eigen::Matrix<double, 3, 2>& _TangentBasisVectors);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Vector6d PLK0;
    Eigen::Matrix<double, 3, 2> TangentBasisVectors;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class ParaLineTranslationFactor : public ceres::SizedCostFunction<2, 7, 2, 2>
{
  public:
    ParaLineTranslationFactor(const Eigen::Vector4d &_pts_i, const Vector6d& _PLK0, const Eigen::Matrix<double, 3, 2>& _TangentBasisVectors);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Vector6d PLK0;
    Eigen::Matrix<double, 3, 2> TangentBasisVectors;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class ParaLineRotationFactor : public ceres::SizedCostFunction<1, 7, 2>
{
public:
    ParaLineRotationFactor(const Eigen::Vector4d &_pts_i, const Vector6d& _PLK0, const Eigen::Matrix<double, 3, 2>& _TangentBasisVectors);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Vector6d PLK0;
    Eigen::Matrix<double, 3, 2> TangentBasisVectors;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
