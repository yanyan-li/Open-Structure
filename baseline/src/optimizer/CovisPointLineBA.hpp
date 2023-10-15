#ifndef __VENOM_SRC_OPTIMIZER_C_BUNDLEADJUSTMENT__
#define __VENOM_SRC_OPTIMIZER_C_BUNDLEADJUSTMENT__

#include <ceres/ceres.h>

#include "src/manager_env/EnvTrajectory.hpp"
#include "src/optimizer/factor/LineProjectionFactor.hpp"
#include "src/optimizer/parametrization/line_parameterization.hpp"
#include "src/optimizer/parametrization/pose_local_parameterization.hpp"
#include "src/utils/UtilTransformer.hpp"

namespace simulator
{
    namespace optimizer
    {
        class CovisPointLineBA
        {
        public:
            static void optimizer(
                simulator::Trajectory *ptr_robot_trajectory,
                simulator::OptimizationManager &opti_para,
                std::map<int, Eigen::Vector3d> &optimized_mappoints,
                std::map<int, Eigen::Matrix<double, 3, 2>> &optimized_maplines,
                std::map<int, Eigen::Matrix4d> &optimized_poses)
            {
                // set parameters
                constexpr size_t first_kf_id_in_map = 0;
                constexpr int opmization_times = 1;
                constexpr int max_iters = 5;
                constexpr int min_obser_num = 3;
                const float huber_thres = 1.5;
                const float pixel_sigma = 1.5;

                // intrinsic
                double fx, fy, cx, cy;

                fx = ptr_robot_trajectory->cam_intri.fx;
                fy = ptr_robot_trajectory->cam_intri.fy;
                cx = ptr_robot_trajectory->cam_intri.cx;
                cy = ptr_robot_trajectory->cam_intri.cy;

                // construct ceres optim parameters
                ceres::Problem problem;
                ceres::LossFunction *loss_function;
                ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
                loss_function = new ceres::CauchyLoss(1.0);

                // add mappoints
                // double Para_Point_Feature[opti_para.mappoints.size()][3];
                auto iteration_pt = opti_para.mappoints.rbegin();
                double Para_Point_Feature[iteration_pt->first + 1][3];
                for (auto mp = opti_para.mappoints.begin(); mp != opti_para.mappoints.end(); mp++)
                {
                    int mp_id = mp->first;
                    if (opti_para.mappoints[mp_id] == Vec3::Zero())
                        continue;
                    Para_Point_Feature[mp_id][0] = mp->second(0);
                    Para_Point_Feature[mp_id][1] = mp->second(1);
                    Para_Point_Feature[mp_id][2] = mp->second(2);
                    problem.AddParameterBlock(Para_Point_Feature[mp_id], 3);
                }

                // add maplines
                // double Para_Line_Feature[opti_para.maplines.size()][4];
                auto iteration_ml = opti_para.maplines.rbegin();
                double Para_Line_Feature[iteration_ml->first + 1][4];
                for (auto ml = opti_para.maplines.begin(); ml != opti_para.maplines.end(); ml++)
                {
                    int ml_id = ml->first;
                    if (opti_para.maplines[ml_id] == Mat32::Zero())
                        continue;
                    double line_length = (opti_para.maplines[ml_id].col(0) - opti_para.maplines[ml_id].col(1)).norm();
                    if (line_length < 0.01)
                        assert(1 == 0);
                    Eigen::Vector3d StartPoint = ml->second.col(0);
                    Eigen::Vector3d EndPoint = ml->second.col(1);
                    Eigen::Vector3d LineDirection = EndPoint - StartPoint;
                    Eigen::Vector3d LinePlaneNormal = StartPoint.cross(EndPoint);
                    double d = LinePlaneNormal.norm() / LineDirection.norm();
                    LinePlaneNormal = LinePlaneNormal.normalized();
                    LineDirection = LineDirection.normalized();
                    Eigen::Matrix<double, 6, 1> LinePLK = Eigen::Matrix<double, 6, 1>::Zero(); // Zero();
                    LinePLK << d * LinePlaneNormal, LineDirection;
                    // from plk to orth
                    Eigen::Vector4d LineOrth = UtiliLine::plk_to_orth(LinePLK);
#if __VERBOSE__OFF
                    // check plk_to_orth  and  orth_to_plk
                    // maplines: 26.3424, 7.98188, 3.30027, 0.957031, 0.289986,3.30027, 27.5251, 27.5251
                    Eigen::Matrix<double, 6, 1> opti_ml_endpoints = UtiliLine::orth_to_plk(LineOrth);
                    Vector3d v = opti_ml_endpoints.head(3);
                    Vector3d n = opti_ml_endpoints.tail(3);
                    std::cout << "maplines: " << LinePLK.head(3)
                              << ", " << LinePLK.tail(3)
                              << ", " << LinePLK.head(3).norm() / LinePLK.tail(3).norm()
                              << ", " << v
                              << ", " << n
                              << "," << v.norm() / n.norm()
                              << ", " << LinePLK.head(3).norm() / v.norm()
                              << ", " << LinePLK.tail(3).norm() / n.norm()
                              << std::endl;
#endif
                    Para_Line_Feature[ml_id][0] = LineOrth(0);
                    Para_Line_Feature[ml_id][1] = LineOrth(1);
                    Para_Line_Feature[ml_id][2] = LineOrth(2);
                    Para_Line_Feature[ml_id][3] = LineOrth(3);

                    // parameterization for orth
                    ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
                    problem.AddParameterBlock(Para_Line_Feature[ml_id], 4, local_parameterization_line); // p,q
                }

                // std::cout<<"ssdsdfda"<<std::endl;

                // add camera pose
                double Para_Pose[opti_para.kfs.size()][7];
                for (auto kf = opti_para.kfs.begin(); kf != opti_para.kfs.end(); kf++)
                {
                    int kf_id = kf->first;
                    Eigen::Matrix4d Twc = kf->second;
                    Eigen::Matrix3d Rot = Twc.block(0, 0, 3, 3);
                    Eigen::Vector3d Tran = Twc.block(0, 3, 3, 1);
                    Eigen::Quaterniond qua(Rot);

                    Para_Pose[kf_id][0] = Tran(0);
                    Para_Pose[kf_id][1] = Tran(1);
                    Para_Pose[kf_id][2] = Tran(2);
                    Para_Pose[kf_id][3] = qua.x();
                    Para_Pose[kf_id][4] = qua.y();
                    Para_Pose[kf_id][5] = qua.z();
                    Para_Pose[kf_id][6] = qua.w();

                    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                    problem.AddParameterBlock(Para_Pose[kf_id], 7, local_parameterization);
                    if (kf_id == 0)
                        problem.SetParameterBlockConstant(Para_Pose[kf_id]);
                }

                // construct ceres problem
                for (auto asso_mp_mea = opti_para.asso_mp_meas.begin(); asso_mp_mea != opti_para.asso_mp_meas.end(); asso_mp_mea++)
                {
                    int mp_id = asso_mp_mea->first;
                    if (opti_para.mappoints[mp_id] == Vec3::Zero())
                        continue;
                    if (asso_mp_mea->second.size() < min_obser_num) // more than two frames detect this point
                        continue;

                    for (int i = 0; i < asso_mp_mea->second.size(); i++)
                    {
                        int kf_id_i = asso_mp_mea->second[i].first;
                        Eigen::Vector2d mp_obser_i = asso_mp_mea->second[i].second;
                        Eigen::Vector2d mp_obser_i_unit((mp_obser_i(0) - cx) / fx, (mp_obser_i(1) - cy) / fy);

                        PointProjectionFactor *f = new PointProjectionFactor(mp_obser_i_unit);
                        problem.AddResidualBlock(f, loss_function, Para_Pose[kf_id_i], Para_Point_Feature[mp_id]);
                    }
                }

                // construct ceres problem
                for (auto asso_ml_mea = opti_para.asso_ml_meas.begin(); asso_ml_mea != opti_para.asso_ml_meas.end(); asso_ml_mea++)
                {
                    int ml_id = asso_ml_mea->first;
                    if (opti_para.maplines[ml_id] == Mat32::Zero())
                        assert(1 == 0);
                    if (asso_ml_mea->second.size() < min_obser_num) // more than two frames detect this point
                        continue;

                    for (int i = 0; i < asso_ml_mea->second.size(); i++)
                    {
                        int kf_id_i = asso_ml_mea->second[i].first;
                        Eigen::Vector4d ml_obser_i = asso_ml_mea->second[i].second;

                        Eigen::Vector4d mp_obser_i_unit(
                            (ml_obser_i(0) - cx) / fx,
                            (ml_obser_i(1) - cy) / fy,
                            (ml_obser_i(2) - cx) / fx,
                            (ml_obser_i(3) - cy) / fy);

                        lineProjectionFactor *f = new lineProjectionFactor(mp_obser_i_unit);
                        problem.AddResidualBlock(f, loss_function, Para_Pose[kf_id_i], Para_Line_Feature[ml_id]);
                    }
                }

                // slove ceres problem
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::SPARSE_SCHUR;
                options.max_num_iterations = 100;
                options.minimizer_progress_to_stdout = true;
                // options.max_num_iterations = 5;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
#if __VERBOSE__
                std::cout << summary.FullReport() << std::endl;
#endif

                // update ceres results
                for (auto kf = opti_para.kfs.begin(); kf != opti_para.kfs.end(); kf++)
                {
                    int kf_id = kf->first;
                    Eigen::Vector3d Tran(Para_Pose[kf_id][0], Para_Pose[kf_id][1], Para_Pose[kf_id][2]);
                    Eigen::Quaterniond qua(Para_Pose[kf_id][6], Para_Pose[kf_id][3], Para_Pose[kf_id][4], Para_Pose[kf_id][5]);
                    Eigen::Matrix3d Rot = qua.toRotationMatrix();
                    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                    Twc.block(0, 0, 3, 3) = Rot;
                    Twc.block(0, 3, 3, 1) = Tran;

                    // std::cout<<"pose: "<< kf->second<<std::endl<<Twc<<std::endl;
                    //  opti_para.kfs[kf_id] = Twc;
                    optimized_poses[kf_id] = Twc;
                }

                // update mappoints
                for (auto mp = opti_para.mappoints.begin(); mp != opti_para.mappoints.end(); mp++)
                {
                    int mp_id = mp->first;
                    Vec3 optimized_mappoint;
                    optimized_mappoint << Para_Point_Feature[mp_id][0], //= mp->second(0);
                        Para_Point_Feature[mp_id][1],                   //= mp->second(1);
                        Para_Point_Feature[mp_id][2];                   //= mp->second(2);
                    optimized_mappoints[mp_id] = optimized_mappoint;
                    // std::cout<<"original: "<< mp->second(0)<<","<< mp->second(1)<< ","<< mp->second(2)<<std::endl;
                    // std::cout << "optimized: " << optimized_mappoint(0) << "," << optimized_mappoint(1) << "," << optimized_mappoint(2) << std::endl;
                }

                // update maplines
                for (auto asso_ml_mea = opti_para.asso_ml_meas.begin(); asso_ml_mea != opti_para.asso_ml_meas.end(); asso_ml_mea++)
                {
                    //

                    int ml_id = asso_ml_mea->first;
                    if (opti_para.maplines[ml_id] == Mat32::Zero())
                        assert(1 == 0);
                    if (asso_ml_mea->second.size() < min_obser_num) // more than two frames detect this point
                        continue;

                    // std::cout << "optimized_maplines_:" << std::endl;

                    Eigen::Vector4d optimized_mapline_orth;
                    optimized_mapline_orth << Para_Line_Feature[ml_id][0],
                        Para_Line_Feature[ml_id][1],
                        Para_Line_Feature[ml_id][2],
                        Para_Line_Feature[ml_id][3];

                    Eigen::Vector3d endpoint_s = opti_para.maplines[ml_id].col(0);
                    Eigen::Vector3d endpoint_e = opti_para.maplines[ml_id].col(1);

                    Eigen::Matrix<double, 6, 1>
                        opti_ml_plk = UtiliLine::orth_to_plk(optimized_mapline_orth);
                    Vector6d endpoints = UtiliLine::plk_to_endpoints(opti_ml_plk, opti_para.maplines[ml_id]);

                    optimized_maplines[ml_id].block(0, 0, 3, 1) = endpoints.head(3);
                    optimized_maplines[ml_id].block(0, 1, 3, 1) = endpoints.tail(3);

                    std::cout << "ml_id:" << ml_id << "optimized:" << optimized_maplines[ml_id] << std::endl;
                }
                return;
            }
        };
    } // namespace optimizer

} // namespace simulator

#endif // __VENOM_SRC_OPTIMIZER_C_BUNDLEADJUSTMENT__
