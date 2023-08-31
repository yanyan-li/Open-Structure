#include "LineProjectionFactor.hpp"
#include "src/optimizer/parametrization/line_parameterization.hpp"
#include "src/utils/UtilTransformer.hpp"
#include <Eigen/src/Core/Matrix.h>

Eigen::Matrix2d PointProjectionFactor::sqrt_info;
double PointProjectionFactor::sum_t;

PointProjectionFactor::PointProjectionFactor(const Eigen::Vector2d &_obs_i)
    : obs_i(_obs_i){};

/*
  parameters[0]:  Twc
  parameters[1]:  Point3d
*/
bool PointProjectionFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const
{
  Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]);

  Eigen::Vector3d point_3d(parameters[1][0], parameters[1][1],
                           parameters[1][2]);

  Eigen::Matrix3d Rwc(qwc);
  Eigen::Vector3d point_c = qwc.inverse() * (point_3d - twc);

  Eigen::Map<Eigen::Vector2d> residual(residuals);

  double dep_j = point_c.z();
  residual = (point_c / dep_j).head<2>() - obs_i;

  sqrt_info.setIdentity();
  residual = sqrt_info * residual;

  if (jacobians)
  {
    Eigen::Matrix<double, 2, 3> reduce(2, 3);

    reduce << 1. / point_c(2), 0, -point_c(0) / (point_c(2) * point_c(2)), 0,
        1. / point_c(2), -point_c(1) / (point_c(2) * point_c(2));

    if (jacobians[0])
    {
      // std::cout <<"jacobian_ex_pose"<<"\n";
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);

      Eigen::Matrix<double, 3, 6> jaco_pintc_pose;
      jaco_pintc_pose.setZero();
      jaco_pintc_pose.block(0, 0, 3, 3) = -Rwc.inverse(); // Lc_t
      jaco_pintc_pose.block(0, 3, 3, 3) =
          UtiliLine::skew_symmetric(point_3d - twc);

      jacobian_pose.leftCols<6>() = reduce * jaco_pintc_pose;
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_Point(
          jacobians[1]);

      jacobian_Point = reduce * Rwc.inverse();
    }
  }

  // check jacobian
  // std::cout << "---------- check jacobian ----------\n";
  // if (jacobians[0])
  //   std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(
  //                    jacobians[0])
  //             << std::endl
  //             << std::endl;
  // if (jacobians[1])
  //   std::cout << Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>(
  //                    jacobians[1])
  //             << std::endl
  //             << std::endl;

  if (jacobians)
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 9> num_jacobian;
    for (int k = 0; k < 9; k++)
    {
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                            parameters[0][5]);

      Eigen::Vector3d point_3d(parameters[1][0], parameters[1][1],
                               parameters[1][2]);

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        Pi += delta;
      else if (a == 1)
        Qi = Qi * UtiliLine::deltaQ(delta);
      else if (a == 2)
        point_3d += delta;

      Eigen::Vector3d point_c = Qi.inverse() * (point_3d - Pi);

      double dep_j = point_c.z();

      Eigen::Vector2d tmp_residual;
      tmp_residual =
          (point_c / dep_j).head<2>() - obs_i; // 误差 = 坐标重投影误差
      tmp_residual = sqrt_info * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.leftCols<6>() = num_jacobian.leftCols<6>();
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
      jacobian_point = num_jacobian.rightCols<3>(); //.rightCols<2>();
    }
  }
  // std::cout << "num_jacobian pose:\n"
  //           << num_jacobian.block(0, 0, 2, 6) << "\n"
  //           << std::endl;

  // std::cout << "num_jacobian point:\n"
  //           << num_jacobian.block(0, 6, 2, 3) << "\n"
  //           << std::endl;
  return true;
}

Eigen::Matrix2d lineProjectionFactor::sqrt_info;
double lineProjectionFactor::sum_t;

lineProjectionFactor::lineProjectionFactor(const Eigen::Vector4d &_obs_i)
    : obs_i(_obs_i){};

/*
  parameters[0]:  Twc
  parameters[1]:  line_orth
*/
bool lineProjectionFactor::Evaluate(double const *const *parameters,
                                    double *residuals,
                                    double **jacobians) const
{
  Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]);

  Eigen::Vector4d line_orth(parameters[1][0], parameters[1][1],
                            parameters[1][2], parameters[1][3]);

  Vector6d line_w = UtiliLine::orth_to_plk(line_orth);
  // from
  Eigen::Matrix3d Rwc(qwc);
  Vector6d line_c = UtiliLine::plk_from_pose(line_w, Rwc, twc);

  // 直线的投影矩阵K为单位阵
  Eigen::Vector3d nc = line_c.head(3);
  double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
  double l_sqrtnorm = sqrt(l_norm);
  double l_trinorm = l_norm * l_sqrtnorm;

  double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
  double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual(0) = e1 / l_sqrtnorm;
  residual(1) = e2 / l_sqrtnorm;

  //    std::cout <<"---- sqrt_info: ------"<< sqrt_info << std::endl;
  sqrt_info.setIdentity();
  residual = sqrt_info * residual;

  // std::cout << residual <<"\n";
  if (jacobians)
  {
    Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
    jaco_e_l << (obs_i(0) / l_sqrtnorm - nc(0) * e1 / l_trinorm),
        (obs_i(1) / l_sqrtnorm - nc(1) * e1 / l_trinorm), 1.0 / l_sqrtnorm,
        (obs_i(2) / l_sqrtnorm - nc(0) * e2 / l_trinorm),
        (obs_i(3) / l_sqrtnorm - nc(1) * e2 / l_trinorm), 1.0 / l_sqrtnorm;

    jaco_e_l = sqrt_info * jaco_e_l;

    Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
    jaco_l_Lc.setZero();
    jaco_l_Lc.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 2, 6> jaco_e_Lc;
    jaco_e_Lc = jaco_e_l * jaco_l_Lc;
    // std::cout <<jaco_e_Lc<<"\n\n";
    // std::cout << "jacobian_calculator:" << std::endl;

    if (jacobians[0])
    {
      // std::cout <<"jacobian_ex_pose"<<"\n";
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);

      Vector3d nb = line_w.head(3);
      Vector3d db = line_w.tail(3);
      Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
      jaco_Lc_ex.setZero();
      jaco_Lc_ex.block(0, 0, 3, 3) =
          Rwc.transpose() * UtiliLine::skew_symmetric(db); // Lc_t
      jaco_Lc_ex.block(0, 3, 3, 3) = UtiliLine::skew_symmetric(
          Rwc.transpose() *
          (nb + UtiliLine::skew_symmetric(db) * twc)); // Lc_theta
      jaco_Lc_ex.block(3, 3, 3, 3) =
          UtiliLine::skew_symmetric(Rwc.transpose() * db);

      jacobian_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
          jacobian_lineOrth(jacobians[1]);

      Eigen::Matrix<double, 6, 6> invTwc;
      invTwc << Rwc.transpose(),
          -Rwc.transpose() * UtiliLine::skew_symmetric(twc),
          Eigen::Matrix3d::Zero(), Rwc.transpose();
      // std::cout<<invTwc<<"\n";

      Vector3d nw = line_w.head(3);
      Vector3d vw = line_w.tail(3);
      Vector3d u1 = nw / nw.norm();
      Vector3d u2 = vw / vw.norm();
      Vector3d u3 = u1.cross(u2);
      Vector2d w(nw.norm(), vw.norm());
      w = w / w.norm();

      Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
      jaco_Lw_orth.setZero();
      jaco_Lw_orth.block(3, 0, 3, 1) = w(1) * u3;
      jaco_Lw_orth.block(0, 1, 3, 1) = -w(0) * u3;
      jaco_Lw_orth.block(0, 2, 3, 1) = w(0) * u2;
      jaco_Lw_orth.block(3, 2, 3, 1) = -w(1) * u1;
      jaco_Lw_orth.block(0, 3, 3, 1) = -w(1) * u1;
      jaco_Lw_orth.block(3, 3, 3, 1) = w(0) * u2;

      // std::cout<<jaco_Lw_orth<<"\n";

      jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
    }
  }

  //   check jacobian
  //  std::cout << "---------- check jacobian ----------\n";
  //  if (jacobians[0])
  //    std::cout << "analy_jacobian pose:\n"
  //              << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(
  //                     jacobians[0])
  //              << std::endl
  //              << std::endl;
  //  if (jacobians[1])
  //    std::cout << "analy_jacobian line:\n"
  //              << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(
  //                     jacobians[1])
  //              << std::endl
  //              << std::endl;
  //  const double eps = 1e-6;
  //  Eigen::Matrix<double, 2, 10> num_jacobian =
  //      Eigen::Matrix<double, 2, 10>::Zero();
  //  for (int k = 0; k < 10; k++) {
  //    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1],
  //    parameters[0][2]); Eigen::Quaterniond Qi(parameters[0][6],
  //    parameters[0][3], parameters[0][4],
  //                          parameters[0][5]);

  //    Eigen::Vector4d line_orth(parameters[1][0], parameters[1][1],
  //                              parameters[1][2], parameters[1][3]);
  //    ceres::LocalParameterization* local_parameterization_line =
  //        new LineOrthParameterization();

  //    int a = k / 3, b = k % 3;
  //    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

  //    if (a == 0)
  //      Pi += delta;
  //    else if (a == 1)
  //      Qi = Qi * UtiliLine::deltaQ(delta);
  //    else if (a == 2) {  // line orth的前三个元素
  //      Eigen::Vector4d line_new;
  //      Eigen::Vector4d delta_l;
  //      delta_l << delta, 0.0;
  //      local_parameterization_line->Plus(line_orth.data(), delta_l.data(),
  //                                        line_new.data());
  //      line_orth = line_new;
  //    } else if (a == 3) {  // line orth的最后一个元素
  //      Eigen::Vector4d line_new;
  //      Eigen::Vector4d delta_l;
  //      delta_l.setZero();
  //      delta_l[3] = delta.x();
  //      local_parameterization_line->Plus(line_orth.data(), delta_l.data(),
  //                                        line_new.data());
  //      line_orth = line_new;
  //    }

  //    Vector6d line_w = UtiliLine::orth_to_plk(line_orth);

  //    Eigen::Matrix3d Rot(Qi);
  //    Vector6d line_c = UtiliLine::plk_from_pose(line_w, Rot, Pi);

  //    // 直线的投影矩阵K为单位阵
  //    Eigen::Vector3d nc = line_c.head(3);
  //    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
  //    double l_sqrtnorm = sqrt(l_norm);
  //    double l_trinorm = l_norm * l_sqrtnorm;

  //    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
  //    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
  //    Eigen::Vector2d tmp_residual;
  //    tmp_residual(0) = e1 / l_sqrtnorm;
  //    tmp_residual(1) = e2 / l_sqrtnorm;
  //    tmp_residual = sqrt_info * tmp_residual;
  //    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  //  }

  //  if (jacobians[1]) {
  //    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
  //    jacobian_lineOrth(
  //        jacobians[1]);
  //    std::cout << "jac" << num_jacobian.template block<2, 4>(0, 6) <<
  //    std::endl; jacobian_lineOrth =
  //        num_jacobian.template block<2, 4>(0, 6);  // (0, 6, 2, 4);
  //  }
  //  std::cout << "num_jacobian pose:\n"
  //            << num_jacobian.block(0, 0, 2, 6) << "\n"
  //            << std::endl;

  //  std::cout << "num_jacobian line:\n"
  //            << num_jacobian.block(0, 6, 2, 4) << "\n"
  //            << std::endl;

  return true;
}

Eigen::Vector2d residual_from_paraline_pose(const Eigen::Matrix3d &Rwc,
                                            const Eigen::Vector3d &twc,
                                            const Vector6d &line_w,
                                            const Eigen::Vector4d &obs_i)
{
  Vector6d line_c = UtiliLine::plk_from_pose(line_w, Rwc, twc);

  // 直线的投影矩阵K为单位阵
  Eigen::Vector3d nc = line_c.head(3);
  double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
  double l_sqrtnorm = sqrt(l_norm);
  double l_trinorm = l_norm * l_sqrtnorm;

  double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
  double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);

  Eigen::Vector2d residual;
  residual(0) = e1 / l_sqrtnorm;
  residual(1) = e2 / l_sqrtnorm;

  return residual;
}

Eigen::Matrix2d ParaLineProjectionFactor::sqrt_info;
double ParaLineProjectionFactor::sum_t;

ParaLineProjectionFactor::ParaLineProjectionFactor(
    const Eigen::Vector4d &_obs_i, const Vector6d &_PLK0, const Eigen::Matrix<double, 3, 2> &_TangentBasisVectors)
    : obs_i(_obs_i), PLK0(_PLK0), TangentBasisVectors(_TangentBasisVectors){};

/*
  parameters[0]:  Twc
  parameters[1]:  line_orth
*/
bool ParaLineProjectionFactor::Evaluate(double const *const *parameters,
                                        double *residuals,
                                        double **jacobians) const
{
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  {
    Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4],
                           parameters[0][5]);

    Eigen::Vector4d para_line_4(parameters[1][0], parameters[1][1],
                                parameters[2][0], parameters[2][1]);
    // std::cout<<"para_line_4:"<< para_line_4<<std::endl;

    // Vector3d LinePlaneNormal = PLK0.head(3);
    // Vector3d LineDirection = PLK0.tail(3);
    // double direction_move0 = para_line_4(0);
    // double direction_move1 = para_line_4(1);
    // Eigen::Vector3d direction_move = direction_move0 * TangentBasisVectors.col(0) + direction_move1 * TangentBasisVectors.col(1);
    // LineDirection = (LineDirection + direction_move).normalized();
    // LinePlaneNormal = (LinePlaneNormal - LineDirection * (LineDirection.transpose() * LinePlaneNormal)).normalized();

    // Vector6d plk_new;
    // plk_new.head(3) = LinePlaneNormal;
    // plk_new.tail(3) = LineDirection_new;
    // PLK0(plk_new);

    Vector6d line_w = UtiliLine::ParaLine2PLK(PLK0, para_line_4, TangentBasisVectors);
    residual = residual_from_paraline_pose(qwc.toRotationMatrix(), twc,
                                           line_w, obs_i);

    // std::cout<<"residual:"<<residual<<std::endl;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
  }

  if (jacobians)
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 10> num_jacobian =
        Eigen::Matrix<double, 2, 10>::Zero();
    for (int k = 0; k < 10; k++)
    {
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                            parameters[0][4], parameters[0][5]);

      Eigen::Vector4d para_line_4(parameters[1][0], parameters[1][1],
                                  parameters[2][0], parameters[2][1]);

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0 && jacobians[0])
        Pi += delta;
      else if (a == 1 && jacobians[0])
        Qi = Qi * UtiliLine::deltaQ(delta);
      else if (a == 2)
      { // line orth的前三个元素
        if (b == 0 || b == 1)
        {
          if (jacobians[1])
            para_line_4.head(2) += Eigen::Vector2d(delta.x(), delta.y());
        }
        else if (b == 2)
        {
          if (jacobians[2])
            para_line_4[2] += delta.z();
        }
      }
      else if (a == 3 && jacobians[2])
      { // line orth的最后一个元素
        para_line_4[3] += delta.x();
      }

      Vector6d line_w_new = UtiliLine::ParaLine2PLK(PLK0, para_line_4, TangentBasisVectors);
      Eigen::Vector2d tmp_residual = residual_from_paraline_pose(
          Qi.toRotationMatrix(), Pi, line_w_new, obs_i);
      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.leftCols<6>() = num_jacobian.leftCols<6>();
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
          jacobian_lineOrth_shared(jacobians[1]);
      jacobian_lineOrth_shared = num_jacobian.block(0, 6, 2, 2); //.rightCols<2>();
    }
    if (jacobians[2])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
          jacobian_lineOrth_unique(jacobians[2]);
      jacobian_lineOrth_unique = num_jacobian.rightCols<2>();
    }
  }

  // update PLK0;

  return true;
}

Eigen::Matrix2d ParaLineTranslationFactor::sqrt_info;
double ParaLineTranslationFactor::sum_t;

ParaLineTranslationFactor::ParaLineTranslationFactor(
    const Eigen::Vector4d &_obs_i, const Vector6d &_PLK0, const Eigen::Matrix<double, 3, 2> &_TangentBasisVectors)
    : obs_i(_obs_i), PLK0(_PLK0), TangentBasisVectors(_TangentBasisVectors){};

/*
  parameters[0]:  Twc
  parameters[1]:  line_orth
*/
bool ParaLineTranslationFactor::Evaluate(double const *const *parameters,
                                         double *residuals,
                                         double **jacobians) const
{
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  {
    Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector4d para_line_4(parameters[1][0], parameters[1][1],
                                parameters[2][0], parameters[2][1]);
    // std::cout<<"para_line_4:"<< para_line_4<<std::endl;

    Vector6d line_w = UtiliLine::ParaLine2PLK(PLK0, para_line_4, TangentBasisVectors);
    residual = residual_from_paraline_pose(qwc.toRotationMatrix(), twc,
                                           line_w, obs_i);

    // std::cout<<"residual:"<<residual<<std::endl;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
  }

  if (jacobians)
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 10> num_jacobian =
        Eigen::Matrix<double, 2, 10>::Zero();
    for (int k = 0; k < 10; k++)
    {
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                            parameters[0][4], parameters[0][5]);

      Eigen::Vector4d para_line_4(parameters[1][0], parameters[1][1],
                                  parameters[2][0], parameters[2][1]);

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0 && jacobians[0])
        Pi += delta;
      else if (a == 1 && jacobians[0])
        Qi = Qi; // * UtiliLine::deltaQ(delta);
      else if (a == 2)
      { // line orth的前三个元素
        if (b == 0 || b == 1)
        {
          if (jacobians[1])
            para_line_4.head(3) += delta;
        }
        else if (b == 2)
        {
          if (jacobians[2])
            para_line_4.head(3) += delta;
        }
      }
      else if (a == 3 && jacobians[2])
      { // line orth的最后一个元素
        para_line_4[3] += delta.x();
      }

      Vector6d line_w_new = UtiliLine::ParaLine2PLK(PLK0, para_line_4, TangentBasisVectors);
      Eigen::Vector2d tmp_residual = residual_from_paraline_pose(
          Qi.toRotationMatrix(), Pi, line_w_new, obs_i);
      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.leftCols<3>() = num_jacobian.leftCols<3>();
      jacobian_pose.rightCols<4>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
          jacobian_lineOrth_shared(jacobians[1]);
      jacobian_lineOrth_shared = num_jacobian.block(0, 6, 2, 2); //.rightCols<2>();
    }
    if (jacobians[2])
    {
      Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
          jacobian_lineOrth_unique(jacobians[2]);
      jacobian_lineOrth_unique = num_jacobian.rightCols<2>();
    }
  }
  return true;
}

double residual_from_paraline_rotation(const Eigen::Matrix3d &Rwc,
                                       const Eigen::Vector2d &line_d_move,
                                       const Vector6d &plk,
                                       const Eigen::Matrix<double, 3, 2> &TangentBasisVectors,
                                       const Eigen::Vector4d &obs_i)
{

  Eigen::Vector3d LinePlaneNormal = plk.head(3).normalized();
  Eigen::Vector3d LineDirection = plk.tail(3).normalized();

  double direction_move0 = line_d_move(0);
  double direction_move1 = line_d_move(1);

  Eigen::Vector3d direction_move = direction_move0 * TangentBasisVectors.col(0) + direction_move1 * TangentBasisVectors.col(1);
  Eigen::Vector3d LineDirection_new = (LineDirection + direction_move).normalized();

  // todo:
  //  Eigen::Vector3d obs_line_direction_norm = E igen::Vector3d(obs_i(0)* obs_depth(0), obs_i(1)* obs_depth(0), obs_depth(0))\
    //                                            - Eigen::Vector3d(obs_i(2)*obs_depth(1), obs_i(3)*obs_depth(1), obs_depth(1));

  Eigen::Vector3d obs_line_plane_norm = Eigen::Vector3d(obs_i(0), obs_i(1), 1).cross(Eigen::Vector3d(obs_i(2), obs_i(3), 1));

  Eigen::Vector3d obs_line_plane_norm_w = Rwc * obs_line_plane_norm;
  obs_line_plane_norm_w = obs_line_plane_norm_w.normalized();

  // Eigen::Vector3d obs_line_direction_norm_w = Rwc*obs_line_direction_norm.normalized();
  // obs_line_direction_norm_w = obs_line_direction_norm_w.normalized();

  //    std::cout<<"obs_line_d:"<<obs_line_d(0)<<","<<obs_line_d(1)<<","<<obs_line_d(2)<<std::endl;
  //    std::cout<<"obs_line_w:"<<obs_line_w(0)<<","<<obs_line_w(1)<<","<<obs_line_w(2)<<std::endl;
  //    std::cout<<"LineDirection_new:"<<LineDirection_new(0)<<","<<LineDirection_new(1)<<","<<LineDirection_new(2)<<std::endl;

  double residual = 0.1 * obs_line_plane_norm_w.transpose() * LineDirection_new;
  // std::cout << "obs_line_d:" << residual << "," << std::endl;

  return residual;
}

Eigen::Matrix2d ParaLineRotationFactor::sqrt_info;
double ParaLineRotationFactor::sum_t;

// ParaLineRotationFactor::ParaLineRotationFactor(
//         const Eigen::Vector4d& _obs_i, const Eigen::Vector2d& _obs_depth, const Vector6d& _PLK0, const Eigen::Matrix<double, 3, 2>& _TangentBasisVectors)
//         : obs_i(_obs_i), PLK0(_PLK0), TangentBasisVectors(_TangentBasisVectors), obs_depth(_obs_depth){};
ParaLineRotationFactor::ParaLineRotationFactor(
    const Eigen::Vector4d &_obs_i, const Vector6d &_PLK0, const Eigen::Matrix<double, 3, 2> &_TangentBasisVectors)
    : obs_i(_obs_i), PLK0(_PLK0), TangentBasisVectors(_TangentBasisVectors){};

/*
  parameters[0]:  Twc
  parameters[1]:  line_orth
*/
bool ParaLineRotationFactor::Evaluate(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians) const
{

  {
    Eigen::Vector3d twc(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qwc(parameters[0][6], parameters[0][3], parameters[0][4],
                           parameters[0][5]);
    Eigen::Vector2d para_line_2(parameters[1][0], parameters[1][1]);
    // std::cout<<"para_line_4:"<< para_line_4<<std::endl;
    (*residuals) = residual_from_paraline_rotation(qwc.toRotationMatrix(), para_line_2, PLK0, TangentBasisVectors, obs_i);
    // std::cout<<"residual:"<<residual<<std::endl;
    sqrt_info.setIdentity();
  }
  if (jacobians)
  {
    const double eps = 1e-6;
    Eigen::Matrix<double, 1, 8> num_jacobian =
        Eigen::Matrix<double, 1, 8>::Zero();
    for (int k = 0; k < 8; k++)
    {
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3],
                            parameters[0][4], parameters[0][5]);

      Eigen::Vector2d para_line_2(parameters[1][0], parameters[1][1]);
      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
      // Eigen::Vector3d delta_x = Eigen::Vector3d(0, 0, 0);
      //      if (a == 0 && jacobians[0])
      //          //Pi += delta_x;
      //  int x = 0;
      //      // Pi += Eigen::Vector3d(0,0,0);
      //      else
      if (a == 1 && jacobians[0])
        Qi = Qi * UtiliLine::deltaQ(delta);
      else if (a == 2 && jacobians[1])
      {
        if (b == 0)
          para_line_2(0) += eps;
        if (b == 1)
          para_line_2(1) += eps;
      }
      double tmp_residual = residual_from_paraline_rotation(Qi.toRotationMatrix(), para_line_2, PLK0, TangentBasisVectors, obs_i);

      double residual = (*residuals);
      num_jacobian(0, k) = (tmp_residual - residual) / eps;
    }

    if (jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(
          jacobians[0]);
      jacobian_pose.leftCols<3>().setZero();
      jacobian_pose.block(0, 3, 1, 3) = num_jacobian.block(0, 3, 1, 3); // leftCols<6>();
      // jacobian_pose.leftCols<6>() = num_jacobian.leftCols<6>();
      jacobian_pose.rightCols<1>().setZero();
    }
    if (jacobians[1])
    {
      Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>>
          jacobian_lineOrth_shared(jacobians[1]);
      jacobian_lineOrth_shared = num_jacobian.rightCols<2>();
    }
  }
  return true;
}