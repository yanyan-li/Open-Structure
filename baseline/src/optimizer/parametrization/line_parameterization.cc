#include "line_parameterization.hpp"
#include "src/utils/UtilTransformer.hpp"

bool LineOrthParameterization::Plus(const double* x,
                                    const double* delta,
                                    double* x_plus_delta) const {
  // ref: 2001, Adrien Bartol,Peter Sturm ,Structure-From-Motion Using Lines:
  // Representation, Triangulation and Bundle Adjustment

  // theta --> U,  phi --> W
  Eigen::Map<const Eigen::Vector3d> theta(x);
  double phi = *(x + 3);
  // Vector3d theta = orth.head(3);
  // double phi = orth[3];
  double s1 = sin(theta[0]); // x, varphi
  double c1 = cos(theta[0]);
  double s2 = sin(theta[1]); // y theta
  double c2 = cos(theta[1]);
  double s3 = sin(theta[2]); // z phi
  double c3 = cos(theta[2]);
  // from eular to rotation
  // cos θ*cos φ,  sin ψ sin θ cos φ − cos ψ sin φ cos ψ sin θ cos φ + sin ψ sin φ
  // cos θ sin φ sin ψ sin θ sin φ + cos ψ cos φ cos ψ sin θ sin φ − sin ψ cos φ
  // − sin θ sin ψ cos θ cos ψ cos θ
  Eigen::Matrix3d R;
  R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, c2 * s3,
      s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, -s2, s1 * c2, c1 * c2;
  double w1 = cos(phi);
  double w2 = sin(phi);

  // update
  Eigen::Map<const Eigen::Vector3d> _delta_theta(delta);
  double _delta_phi = *(delta + 3);
  Eigen::Matrix3d Rz;
  Rz << cos(_delta_theta(2)), -sin(_delta_theta(2)), 0, sin(_delta_theta(2)),
      cos(_delta_theta(2)), 0, 0, 0, 1;

  Eigen::Matrix3d Ry;
  Ry << cos(_delta_theta(1)), 0., sin(_delta_theta(1)), 0., 1., 0.,
      -sin(_delta_theta(1)), 0., cos(_delta_theta(1));

  Eigen::Matrix3d Rx;
  Rx << 1., 0., 0., 0., cos(_delta_theta(0)), -sin(_delta_theta(0)), 0.,
      sin(_delta_theta(0)), cos(_delta_theta(0));
  R = R * Rx * Ry * Rz;

  Eigen::Matrix2d W;
  W << w1, -w2, w2, w1;
  Eigen::Matrix2d delta_W;
  delta_W << cos(_delta_phi), -sin(_delta_phi), sin(_delta_phi),
      cos(_delta_phi);
  W = W * delta_W;

  // U' -- > theta'. W' --> phi'
  Eigen::Map<Eigen::Vector3d> theta_pluse(
      x_plus_delta);  // double 指针 转为eigen数组
  double* phi_plus(x_plus_delta + 3);

  Eigen::Vector3d u1 = R.col(0);
  Eigen::Vector3d u2 = R.col(1);
  Eigen::Vector3d u3 = R.col(2);
//   theta_pluse[0] = atan2(u2(2), u3(2));
//   theta_pluse[1] = asin(-u1(2));
//   theta_pluse[2] = atan2(u1(1), u1(0));

  double sy = sqrt(u1(0)*u1(0)+u1(1)*u1(1));
    //float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
  // std::cout<<"sy:"<<sy<<std::endl;
  assert(sy > 1e-6);
    // todo:: use SO3
  //   // orth[0] = atan2(u2(2), u3(2));
  //   // orth[1] = asin(-u1(2));
  //   // orth[2] = atan2(u1(1), u1(0));
  theta_pluse[0] = atan2(u2(2), u3(2));
  theta_pluse[1] = atan2(-u1(2), sy); // asin(-u1(2));
  theta_pluse[2] = atan2(u1(1), u1(0));

  *phi_plus = asin(W(1, 0));
  return true;
}
bool LineOrthParameterization::ComputeJacobian(const double* x,
                                               double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> j(jacobian);
  j.setIdentity();

  return true;
}

bool ParaLineOrthParameterization::Plus(const double *x,
                                        const double *delta,
                                        double *x_plus_delta) const
{
  // theta --> U,  phi --> W
  double theta = *(x); // theta varphi alpha
  double phi = *(x + 1); // beta
  // Vector3d theta = orth.head(3);
  // double phi = orth[3];
  double w1 = cos(phi);
  double w2 = sin(phi);

  // update
  double _delta_theta = *(delta);
  double _delta_phi = *(delta + 1);

  Eigen::Matrix2d W;
  W << w1, -w2, w2, w1;
  Eigen::Matrix2d delta_W;
  delta_W << cos(_delta_phi), -sin(_delta_phi), sin(_delta_phi),
      cos(_delta_phi);
  W = W * delta_W;

  // U' -- > theta'. W' --> phi'
   double *theta_pluse (x_plus_delta);  
      
   double* phi_plus(x_plus_delta + 1);

  *theta_pluse = theta + _delta_theta;

  *phi_plus = asin(W(1, 0));

  return true;
}
bool ParaLineOrthParameterization::ComputeJacobian(const double* x,
                                               double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> j(jacobian);
  j.setIdentity();

  return true;
}
