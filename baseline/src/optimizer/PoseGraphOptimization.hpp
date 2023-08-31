#ifndef __VENOM_SRC_OPTIMIZER_POSE_GRAPH_OPTIMIZATION__
#define __VENOM_SRC_OPTIMIZER_POSE_GRAPH_OPTIMIZATION__

#include <ceres/ceres.h>

#include "src/manager_env/EnvTrajectory.hpp" 
#include "src/optimizer/factor/PoseGraphSE3Factor.hpp"
#include "src/optimizer/factor/PoseGraphSO3Factor.hpp"
#include "src/utils/IOFuntion.hpp"
#include "src/utils/UtilTransformer.hpp"

namespace simulator {
namespace optimizer {
class PoseGraphOptimization {
 public:
  static void optimizer(IO::MapOfPoses &poses,
                        IO::VectorOfConstraints &constraints,
                        std::vector<Mat4> &Twcs_posegraph) {
    // set
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    ceres::LocalParameterization *quaternion_local =
        new ceres::EigenQuaternionParameterization;

    auto former_vertex = poses.begin();
    for (auto mit = std::next(poses.begin()); mit != poses.end();
         former_vertex = mit, mit++) {
      if ((former_vertex)->first != ((mit)->first - 1)) continue;

      // Compute the relative T
      Eigen::Quaterniond q_former_inverse =
          (former_vertex)->second.q.conjugate();
      Eigen::Quaterniond q_former_mit = q_former_inverse * (mit)->second.q;

      Eigen::Matrix<double, 3, 1> p_former_mit =
          q_former_inverse * ((mit)->second.p - (former_vertex)->second.p);
      const Eigen::Matrix<double, 6, 6> sqrt_infomation =
          Eigen::Matrix<double, 6, 6>::Identity();

      ceres::CostFunction *cost_function = Optimizer::PoseGraphSE3Factor::Create(
          IO::Pose3d{p_former_mit, q_former_mit}, sqrt_infomation);

      problem.AddResidualBlock(
          cost_function, nullptr, (former_vertex)->second.p.data(),
          (former_vertex)->second.q.coeffs().data(), (mit)->second.p.data(),
          (mit)->second.q.coeffs().data());
    }

    for (auto mit = poses.begin(); mit != poses.end(); mit++) {
      problem.AddParameterBlock(mit->second.p.data(), 3);
      problem.AddParameterBlock(mit->second.q.coeffs().data(), 4,
                                quaternion_local);
    }

    for (int i = 0; i < constraints.size(); i++) {
      int id_begin = constraints.at(i).id_begin;
      int id_end = constraints.at(i).id_end;

      if (i % 3 != 0) {
        continue;
      } else {
        ceres::CostFunction *cost_function =
            Optimizer::PoseGraphSE3Factor::Create(
                IO::Pose3d{constraints.at(i).t_be.p, constraints.at(i).t_be.q},
                constraints.at(i).information);

        problem.AddResidualBlock(
            cost_function, nullptr, poses.at(id_begin).p.data(),
            poses.at(id_begin).q.coeffs().data(), poses.at(id_end).p.data(),
            poses.at(id_begin).q.coeffs().data());

        problem.SetParameterization(poses.at(id_begin).q.coeffs().data(),
                                    quaternion_local);
        problem.SetParameterization(poses.at(id_end).q.coeffs().data(),
                                    quaternion_local);

        if (id_begin == 0) {
          problem.SetParameterBlockConstant(poses.at(id_begin).p.data());
          problem.SetParameterBlockConstant(
              poses.at(id_begin).q.coeffs().data());
        }
      }

      if (i % 2 == 0) {
        continue;
      } else {
        const Eigen::Matrix<double, 3, 3> sqrt_information_rot =
            constraints.at(i).information.template block<3, 3>(0, 0);

        // std::cout<<sqrt_information<<std::endl;
        // std::cout<<"relative rot:"<<ptr_so3_edge->rot_.x()<<std::endl;
        ceres::CostFunction *cost_function =
            Optimizer::PoseGraphSO3Factor::Create(constraints.at(i).t_be.q,
                                                  1000 * sqrt_information_rot);

        problem.AddResidualBlock(cost_function, nullptr,
                                 poses.at(id_begin).q.coeffs().data(),
                                 poses.at(id_end).q.coeffs().data());

        // std::cout<<"residual block:"<<ptr_so3_edge->a_->rot_.x()<<std::endl;
        problem.SetParameterization(poses.at(id_begin).q.coeffs().data(),
                                    quaternion_local);
        problem.SetParameterization(poses.at(id_end).q.coeffs().data(),
                                    quaternion_local);

        if (id_begin == 0) {
          problem.SetParameterBlockConstant(
              poses.at(id_begin).q.coeffs().data());
        }
      }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "******start rotation********" << std::endl;
    std::cout << summary.FullReport() << std::endl;

    // pose_graph.writePoseGraph("vertex_opti_rot.txt");
  }
};
}  // namespace optimizer

}  // namespace simulator
   // problem
#endif  //_VENOM_SRC_OPTIMIZER_POSE_GRAPH_OPTIMIZATION__
