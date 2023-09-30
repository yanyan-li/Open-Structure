#ifndef __VENOM_SRC_UTIL_UtilTransformer_HPP__
#define __VENOM_SRC_UTIL_UtilTransformer_HPP__

#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace Eigen;

class UtiliLine {
 public:
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

  static Vector4d line_to_orth(Vector6d line) {
    Vector4d orth;
    Vector3d p = line.head(3);
    Vector3d v = line.tail(3);
    Vector3d n = p.cross(v);

    Vector3d u1 = n / n.norm();
    Vector3d u2 = v / v.norm();
    Vector3d u3 = u1.cross(u2);

    // orth[0] = atan2(u2(2), u3(2));
    // orth[1] = asin(-u1(2));
    // orth[2] = atan2(u1(1), u1(0));

    double sy = sqrt(u1(0)*u1(0)+u1(1)*u1(1));
    //float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    //std::cout<<"sy:"<<sy<<std::endl;
    assert(sy > 1e-6);

    orth[0] = atan2(u2(2), u3(2));
    orth[1] = atan2(-u1(2), sy );    // asin(-u1(2));
    orth[2] = atan2(u1(1), u1(0));

    Vector2d w(n.norm(), v.norm());
    w = w / w.norm();
    orth[3] = asin(w(1));

    return orth;
  }

  static Vector6d orth_to_line(Vector4d orth) {
    Vector6d line;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    // todo:: SO3
    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, c2 * s3,
        s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, -s2, s1 * c2, c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1 / w2;  // 原点到直线的距离

    line.head(3) = -R.col(2) * d;
    line.tail(3) = R.col(1);

    return line;
  }

  static Vector4d plk_to_orth(Vector6d plk) {
    Vector4d orth;
    Vector3d n = plk.head(3);
    Vector3d v = plk.tail(3);

    Vector3d u1 = n / n.norm();
    Vector3d u2 = v / v.norm();
    Vector3d u3 = u1.cross(u2);
    //
    double sy = sqrt(u1(0)*u1(0)+u1(1)*u1(1));
    // float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    //  std::cout << "From rotation mat to eular: sy:" << sy << ", n.norm():" << n.norm() << std::endl;
    assert(sy > 1e-6);
    // todo:: use SO3

    // orth[0] = atan2(u2(2), u3(2));
    // orth[1] = asin(-u1(2));
    // orth[2] = atan2(u1(1), u1(0));

    orth[0] = atan2(u2(2), u3(2));
    orth[1] = atan2(-u1(2), sy );    // asin(-u1(2));
    orth[2] = atan2(u1(1), u1(0));

    Vector2d w(n.norm(), v.norm());
    w = w / w.norm();
    orth[3] = asin(w(1));

    return orth;
  }
  // static Vector4d plk_to_orth(Vector6d plk)
  // {
  //   Vector4d orth;
  //   Vector3d n = plk.head(3);
  //   Vector3d v = plk.tail(3);

  //   Vector3d u1 = n / n.norm();
  //   Vector3d u2 = v / v.norm();
  //   Vector3d u3 = u1.cross(u2);

  //   // todo:: use SO3
  //   orth[0] = atan2(u2(2), u3(2));
  //   orth[1] = asin(-u1(2));
  //   orth[2] = atan2(u1(1), u1(0));

  //   Vector2d w(n.norm(), v.norm());
  //   w = w / w.norm();
  //   orth[3] = asin(w(1));

  //   return orth;
  // }

  static Vector6d plk_to_endpoints(const Vector6d& plk_w,
                                   const Vector4d& obs,
                                   const Eigen::Matrix4d& Twc) {
    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);

    Vector6d plk_c = UtiliLine::plk_from_pose(plk_w, Rwc, twc);

    Vector3d pc, nc, vc;
    nc = plk_c.head(3);
    vc = plk_c.tail(3);
    Matrix4d Lc;
    Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

    Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
    Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
    Vector2d ln = (p11.cross(p21)).head(2);  // 直线的垂直方向
    ln = ln / ln.norm();

    Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1),
                            1.0);  // 直线垂直方向上移动一个单位
    Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
    Vector3d cam = Vector3d(0, 0, 0);

    Vector4d pi1 = pi_from_ppp(cam, p11, p12);
    Vector4d pi2 = pi_from_ppp(cam, p21, p22);

    Vector4d e1 = Lc * pi1;
    Vector4d e2 = Lc * pi2;
    e1 = e1 / e1(3);
    e2 = e2 / e2(3);

    Vector3d pts_1(e1(0), e1(1), e1(2));
    Vector3d pts_2(e2(0), e2(1), e2(2));

    Vector6d endpoints;
    endpoints.head(3) = Rwc * pts_1 + twc;
    endpoints.tail(3) = Rwc * pts_2 + twc;
    return endpoints;
  }

  static Vector6d orth_to_plk(Vector4d orth) {
    Vector6d plk;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, c2 * s3,
        s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, -s2, s1 * c2, c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    // double d = w1 / w2;  // 原点到直线的距离

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    // Vector3d Q = -R.col(2) * d;
    // plk.head(3) = Q.cross(v);
    // plk.tail(3) = v;

    return plk;
  }

  static Vector6d paraliorth_to_plk(Vector4d parali_orth)
  {
     Vector6d plk;
    // theta varphi  alpha
    Vector3d theta = parali_orth.head(3);
    // beta
    double phi = parali_orth[3];
    
    //
    double s_theta = sin(theta[0]);
    double c_theta = cos(theta[0]);

    double s_varphi = sin(theta[1]);
    double c_varphi = cos(theta[1]);
    
    double s_alpha = sin(theta[2]);
    double c_alpha = cos(theta[2]); 
    
    Eigen::Vector3d U_2 = Eigen::Vector3d(
      s_theta*c_varphi, s_theta*s_varphi, c_theta);
    Eigen::Vector3d U_1 = Eigen::Vector3d(
      -s_varphi*c_alpha-c_theta*c_varphi*c_alpha,
      c_varphi*c_alpha-c_theta*s_varphi*c_alpha,
      -c_theta*s_varphi*c_alpha+s_theta*s_varphi*s_varphi*c_alpha);   


    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1 / w2;  // 原点到直线的距离

    // Vector3d u1 = R.col(0);
    // Vector3d u2 = R.col(1);

    Vector3d n = w1 * U_1;
    Vector3d v = w2 * U_2;

    plk.head(3) = n;
    plk.tail(3) = v;

    return plk;
  }

  static Eigen::Matrix<double, 3, 2> TangentBasis(const Eigen::Vector3d &g0)
  {
      Vector3d b, c;
      Vector3d a = g0.normalized();
      Vector3d tmp(0, 0, 1);
      if(a == tmp)
          tmp << 1, 0, 0;
      // get perp_base    
      b = (tmp - a * (a.transpose() * tmp)).normalized();
      // std::cout<<"sss:"<< b.transpose()*a<<std::endl;

      // Vector3d  b_new = (tmp - tmp * (a.transpose() * tmp)).normalized();
      // std::cout<<"sss_new:"<< b_new.transpose()*a<<std::endl;
      // get another base
      c = a.cross(b);
      Eigen::Matrix<double, 3, 2> bc;
      bc.block<3, 1>(0, 0) = b;
      bc.block<3, 1>(0, 1) = c;
      
      return bc;
  }

  static void ParaLineUniqueInitialization(const Vector6d &PLK0, const Vector6d &PLK1, Vector2d &unique)
  {
    Eigen::Vector3d LineDirection = PLK0.tail(3).normalized();
    Eigen::Vector3d LinePlaneNormal0 = PLK0.head(3).normalized();
    Eigen::Vector3d LinePlaneNormal1 = PLK1.head(3).normalized();

    double norm_angle = acos(LinePlaneNormal0.dot(LinePlaneNormal1));
    double distance1 = PLK1.head(3).norm() / PLK1.tail(3).norm();

    // Eigen::Matrix<double, 3, 2> TangentBasisVectors = TangentBasis(LineDirection);

    Eigen::Vector2d parali;
    parali(0) = norm_angle; parali(1) = distance1;
  }

  static Vector4d PLK2ParaLine(Vector6d &PLK, Eigen::Matrix<double, 3, 2>& TangentBasisVectors){
    Eigen::Vector3d LineDirection = PLK.tail(3).normalized();
    Eigen::Vector3d LinePlaneNormal = PLK.head(3).normalized();
    double distance = PLK.head(3).norm() / PLK.tail(3).norm();

    TangentBasisVectors = TangentBasis(LineDirection);

    Eigen::Vector4d parali;
    parali(0) = 0; parali(1) = 0;
    parali(2) = 0; parali(3) = distance;

    return parali;
  }

  static Vector4d PLKPLK2ParaLine(Vector6d &PLK0, Vector6d &PLK1){
    Eigen::Vector3d LineDirection = PLK0.tail(3).normalized();
    Eigen::Vector3d LinePlaneNormal0 = PLK0.head(3).normalized();
    Eigen::Vector3d LinePlaneNormal1 = PLK1.head(3).normalized();

    Eigen::Vector3d LinePlaneNormal1_in0s = LinePlaneNormal1 - LineDirection * (LineDirection.transpose().dot(LinePlaneNormal1));
    //  .normalized();
    double distance1 = LinePlaneNormal1_in0s.norm();
    if (false)
    {
          Eigen::Matrix3d R;
          Eigen::Vector3d z_axis = LinePlaneNormal0.cross(LineDirection);
          z_axis = z_axis.normalized();
          R.col(0) = LinePlaneNormal0; // x
          R.col(1) = LineDirection;    // y
          R.col(2) = z_axis;           // z

          Eigen::Vector3d LinePlaneNormal1_in0 = R.transpose() * LinePlaneNormal1;
          LinePlaneNormal1_in0(1, 0) = 0;
          Eigen::Vector3d LinePlaneNormal1_in0s = LinePlaneNormal1_in0.normalized();
          double distance1 = PLK1.head(3).norm() / PLK1.tail(3).norm();
    }

    //double distance1 = LinePlaneNormal1_in0s.norm();

    double norm_angle;
    if(LinePlaneNormal1_in0s(2) > 0)
      norm_angle = acos(LinePlaneNormal1_in0s(0) - 1e-6);
    else
      norm_angle = -acos(LinePlaneNormal1_in0s(0) - 1e-6);
    // std::cout << "LinePlaneNormal1 = " << LinePlaneNormal1.transpose() << std::endl;
    // std::cout << "LinePlaneNormal1_in0 = " << LinePlaneNormal1_in0.transpose() << std::endl;
    // std::cout << "norm_angle = " << norm_angle << std::endl;

    // Eigen::Matrix<double, 3, 2> TangentBasisVectors = TangentBasis(LineDirection);
    Eigen::Vector4d parali;
    parali(0) = 0; parali(1) = 0;
    parali(2) = norm_angle; parali(3) = 1./ distance1;

    return parali;
  }

  static Vector6d ParaLine2PLK(const Vector6d &PLK0, Vector4d &parali, const Eigen::Matrix<double, 3, 2>& TangentBasisVectors){
    Eigen::Vector3d LinePlaneNormal = PLK0.head(3).normalized();
    Eigen::Vector3d LineDirection = PLK0.tail(3).normalized();
    
    double direction_move0 = parali(0);
    double direction_move1 = parali(1);

    Eigen::Vector3d direction_move = direction_move0 * TangentBasisVectors.col(0) + direction_move1 * TangentBasisVectors.col(1);
    Eigen::Vector3d LineDirection_new = (LineDirection + direction_move).normalized();
    Eigen::Vector3d LinePlaneNormal_new =  (LinePlaneNormal- LineDirection_new *(LineDirection_new.transpose()*LinePlaneNormal)).normalized();
    //std::cout<<"direction_move:"<<   direction_move<<std::endl;
    //std::cout<<"paraline2plk:"<<LinePlaneNormal<<", "<<LinePlaneNormal- LinePlaneNormal *(LineDirection_new.transpose()*LinePlaneNormal)<<std::endl;

    double norm_angle = parali(2);
    double distance1 = 1./parali(3);

    Eigen::Vector3d LinePlaneNormal_target(std::cos(norm_angle), 0, std::sin(norm_angle));
    
    Eigen::Matrix3d R;
    Eigen::Vector3d z_axis = LinePlaneNormal_new.cross(LineDirection_new);
    z_axis = z_axis.normalized();
    R.col(0) = LinePlaneNormal_new;
    R.col(1) = LineDirection_new;
    R.col(2) = z_axis;

    LinePlaneNormal_target = (R * LinePlaneNormal_target).normalized();

    Vector6d plk_new;
    plk_new.head(3) = LinePlaneNormal_target * distance1;
    plk_new.tail(3) = LineDirection_new;

    return plk_new;
  }

  static Vector2d ParaLineDirectionToS2(Vector3d& direction) {
    Vector2d s2;  // theta varphi
    std::cout<<"direcion: "<<direction<<std::endl;
    s2(0) = acos(direction.z());
    s2(1) = asin(direction.y() / sin(s2(0)));
    double varphi = acos(direction.x() / sin(s2(0)));

    std::cout<<"varphi:"<<varphi <<", "<< s2(1)<<std::endl;
    assert(varphi == s2(1));
    return s2;
  }

  static double ParaLineSE2ToR1(Vector2d& se2)  //
  {
    // double a = se2/se2.norm();
    se2.normalize();
    
    double beta = asin(se2(1));
    std::cout<<"beta:"<<beta<<std::endl;
    return beta;
  }

  static double ParaLineCycleToR1(Vector3d &normal, Vector2d &parali_s2)
  {
    double varphi, theta;

    theta = parali_s2(0);
    varphi = parali_s2(1);

    /// test
    std::cout<<"theta:"<<theta<<std::endl;
    double alpha_z = asin(normal(2) / sin(theta));
    std::cout << "alpha_z:" << alpha_z << std::endl;
    std::cout << "extimated normal_0: " << -sin(varphi) * cos(alpha_z) - cos(theta) * cos(varphi) * sin(alpha_z) << ". expected:" << normal(0) << std::endl;
    std::cout << "extimated normal_1: " << cos(varphi) * cos(alpha_z) - cos(theta) * sin(varphi) * sin(alpha_z) << ". expected:" << normal(1) << std::endl;
    std::cout << "estimated normal_2: " << sin(theta)*sin(alpha_z)<<". expected:"<< normal(2)<< std::endl;

    return alpha_z;
  }

  //TODO:Yanyan
  static Vector6d ParaLineOrthToPLK(Vector4d parali_orth)
  {
    Vector6d plk;

    Vector3d theta = parali_orth.head(3);  //theta varphi alpha beta
    double phi = parali_orth[3];

    // TODO:
    double s1 = sin(theta[0]);  
    double c1 = cos(theta[0]); 
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R << c2 * c3, s1 * s2 * c3 - c1 * s3, c1 * s2 * c3 + s1 * s3, 
         c2 * s3, s1 * s2 * s3 + c1 * c3, c1 * s2 * s3 - s1 * c3, 
         -s2, s1 * c2, c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    // double d = w1 / w2;  // 原点到直线的距离

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    // Vector3d Q = -R.col(2) * d;
    // plk.head(3) = Q.cross(v);
    // plk.tail(3) = v;

    return plk;

  }
  /*
   三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d =
   -(ax0 + by0 + cz0) 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
   (a,b,c)^T = vector(AO) cross vector(BO)
   d = O.dot(cross(AO,BO))
   */
  static Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector4d pi;
    pi << (x1 - x3).cross(x2 - x3),
        -x3.dot(x1.cross(x2));  // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = -
                                // x3.dot( x1.cross( x2 ) )

    return pi;
  }

  // find parallel lines from a coarse-parallel-line set
  static int get_parallel_candidate(
      std::vector<int /*paramapline_id*/> &mpli_ids,
      std::map<int /*maplines_id*/, Eigen::Matrix<double, 3, 2>> &mplis,
      std::vector<int /*mapline_id*/> &para_inlier,
      std::vector<int /*mapline_id*/> &individure_outlier)
  {
    int max_inlier = 0;
    for (int inter = 0; inter < 100; inter++)
    {
      int num = mpli_ids.size();
      int random_number = rand() % num; // generate a random integer between 0 and n
      Eigen::Vector3d startpoint_seed = mplis[mpli_ids[random_number]].col(0);
      Eigen::Vector3d endpoint_seed = mplis[mpli_ids[random_number]].col(1);
      Eigen::Vector3d line_direction_seed = (startpoint_seed - endpoint_seed).normalized();
      std::vector<int /*id*/> inliers;
      std::vector<int /*id*/> outliers;
      inliers.clear();
      outliers.clear();
      inliers.push_back(mpli_ids[random_number]);

      for (auto &ml_id : mpli_ids)
      {
        // if (mplis[ml_id] ==Eigen::Matrix Mat32::Zero())
        //   continue;
        if (mplis[ml_id].col(0).norm() < 0.001 && mplis[ml_id].col(1).norm() < 0.001)
          continue;

        Eigen::Vector3d startpoint = mplis[ml_id].col(0);
        Eigen::Vector3d endpoint = mplis[ml_id].col(1);
        Eigen::Vector3d line_direction = (startpoint - endpoint).normalized();
        // std::cout<<"seed:"<<random_number<<". "<<line_direction.transpose()*line_direction_seed<<","<<line_direction_seed.transpose()<<", "<<line_direction.transpose()<<std::endl;
        if (ml_id == mpli_ids[random_number])
          continue;
        if (line_direction.transpose() * line_direction_seed > 0.9)
          inliers.push_back(ml_id);
        else if (-line_direction.transpose() * line_direction_seed > 0.9)
        {
          inliers.push_back(ml_id);
          // exchange
          Eigen::Vector3d startpoint = mplis[ml_id].col(0);
          mplis[ml_id].col(0) = mplis[ml_id].col(1);
          mplis[ml_id].col(1) = startpoint;
        }
        else
          outliers.push_back(ml_id);
      }

      if (inliers.size() > max_inlier)
      {
        max_inlier = inliers.size();
        para_inlier.clear();
        individure_outlier.clear();
        std::cout << ">> refresh:" << std::endl;
        for (int j = 0; j < inliers.size(); j++)
        {
          para_inlier.push_back(inliers[j]);
#ifdef __DEBUG__
          Eigen::Vector3d startpoint_seed = mplis[inliers[j]].col(0);
          Eigen::Vector3d endpoint_seed = mplis[inliers[j]].col(1);
          Eigen::Vector3d line_direction_seed = (startpoint_seed - endpoint_seed).normalized();
          std::cout << "inlier direction:" << inliers[j] << ","
                    << line_direction_seed(0) << "," << line_direction_seed(1) << "," << line_direction_seed(2) << std::endl;
#endif
        }

        for (int j = 0; j < outliers.size(); j++)
        {
          individure_outlier.push_back(outliers[j]);
#ifdef __DEBUG__
          Eigen::Vector3d startpoint_seed = mplis[outliers[j]].col(0);
          Eigen::Vector3d endpoint_seed = mplis[outliers[j]].col(1);
          Eigen::Vector3d line_direction_seed = (startpoint_seed - endpoint_seed).normalized();
          std::cout << "direction:" << outliers[j] << line_direction_seed(0) << "," << line_direction_seed(1) << "," << line_direction_seed(2) << std::endl;
#endif
        }
      }
    }
  }

  static Vector3d refine_endpoint(Vector3d& endpoint, Vector6d plk) {
    // TODO:
  }

  // 两平面相交得到直线的plucker 坐标
  static Vector6d pipi_plk(Vector4d pi1, Vector4d pi2) {
    Vector6d plk;
    Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

    plk << dp(0, 3), dp(1, 3), dp(2, 3), -dp(1, 2), dp(0, 2), -dp(0, 1);
    return plk;
  }

  // 获取光心到直线的垂直点
  static Vector3d plucker_origin(Vector3d n, Vector3d v) {
    return v.cross(n) / v.dot(v);
  }

  static Matrix3d skew_symmetric(Vector3d v) {
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
  }

  static Vector3d point_to_pose(Eigen::Matrix3d Rcw,
                                Eigen::Vector3d tcw,
                                Vector3d pt_w) {
    return Rcw * pt_w + tcw;
  }

  // 从相机坐标系到世界坐标系
  static Vector3d poit_from_pose(Eigen::Matrix3d Rcw,
                                 Eigen::Vector3d tcw,
                                 Vector3d pt_c) {
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc * tcw;
    return point_to_pose(Rwc, twc, pt_c);
  }

  static Vector6d line_to_pose(Vector6d line_w,
                               Eigen::Matrix3d Rcw,
                               Eigen::Vector3d tcw) {
    Vector6d line_c;

    Vector3d cp_w, dv_w;
    cp_w = line_w.head(3);
    dv_w = line_w.tail(3);

    Vector3d cp_c = point_to_pose(Rcw, tcw, cp_w);
    Vector3d dv_c = Rcw * dv_w;

    line_c.head(3) = cp_c;
    line_c.tail(3) = dv_c;

    return line_c;
  }

  static Vector6d line_from_pose(Vector6d line_c,
                                 Eigen::Matrix3d Rcw,
                                 Eigen::Vector3d tcw) {
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc * tcw;
    return line_to_pose(line_c, Rwc, twc);
  }

  // 世界坐标系到相机坐标系下
  static Vector6d plk_to_pose(Vector6d plk_w,
                              Eigen::Matrix3d Rcw,
                              Eigen::Vector3d tcw) {
    Vector3d nw = plk_w.head(3);
    Vector3d vw = plk_w.tail(3);

    Vector3d nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
    Vector3d vc = Rcw * vw;

    Vector6d plk_c;
    plk_c.head(3) = nc;
    plk_c.tail(3) = vc;
    return plk_c;
  }

  static Vector6d plk_from_pose(Vector6d plk_c,
                                Eigen::Matrix3d Rcw,
                                Eigen::Vector3d tcw) {
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc * tcw;
    return plk_to_pose(plk_c, Rwc, twc);
  }
};

#endif
