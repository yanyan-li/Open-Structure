/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-28 10:12:15
 * @LastEditTime: 2023-08-28 10:12:17
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /JointFactorGraph/src/optimizer/CovisPointParaLineBA2.cpp
 */
#ifndef __VENOM_SRC_OPTIMIZER_COVISEXTE_GRAPH_BUNDLEADJUSTMENT_2__
#define __VENOM_SRC_OPTIMIZER_COVISEXTE_GRAPH_BUNDLEADJUSTMENT_2__
#include "src/manager_env/EnvTrajectory.hpp"
#include "src/optimizer/factor/LineProjectionFactor.hpp"
#include "src/optimizer/parametrization/line_parameterization.hpp"
#include "src/optimizer/parametrization/pose_local_parameterization.hpp"
// #include "src/optimizer/factor/ParaLineChordalFactor.hpp"
#include "glog/logging.h"
#include "src/manager_env/EnvManager.hpp"
#include "src/utils/UtilTransformer.hpp"
#include <ceres/ceres.h>
#include <cstdlib> // for rand() and RAND_MAX
#include <iostream>
namespace simulator
{
    namespace optimizer
    {
        class CovisPointParalineBA2
        {
        public:
            static void optimizer(
                simulator::Trajectory *ptr_robot_trajectory,
                simulator::OptimizationManager &opti_para,
                std::map<int /*mappoint_id*/, Eigen::Vector3d> &optimized_mappoints,
                std::map<int /*mapline_id*/, Eigen::Matrix<double, 3, 2>> &optimized_maplines,
                std::map<int, Eigen::Matrix4d> &optimized_poses)
            {
                // set parameters
                constexpr size_t first_kf_id_in_map = 0;
                constexpr int opmization_times = 1;
                constexpr int max_iters = 5;
                constexpr int min_obser_num = 3;
                const float huber_thres = 1.5;
                const float pixel_sigma = 1.5;

                double fx;
                double fy;
                double cx;
                double cy;
                fx = ptr_robot_trajectory->cam_intri.fx;
                fy = ptr_robot_trajectory->cam_intri.fy;
                cx = ptr_robot_trajectory->cam_intri.cx;
                cy = ptr_robot_trajectory->cam_intri.cy;

                // parameters
                ceres::Problem problem;
                ceres::LossFunction *loss_function = nullptr;
                ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
                ceres::LocalParameterization *pose_parameterization = new PoseLocalParameterization();
                loss_function = new ceres::CauchyLoss(1.0);

                //----> mappoints
                double Para_Point_Feature[opti_para.mappoints.size()][3];
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

                //----> mapparaline
                std::map<int /*parali_id*/, std::vector<int /*mapline_id*/>> paralineid_mlids;
                std::vector<int> single_line;
                int vanishing_id = 0;
                for (auto &parali_id : opti_para.asso_paralineid_mlids)
                {
                    //---> build ParaLine
                    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
                    // direction
                    std::vector<int> para_inlier;
                    std::vector<int> individual_line;
                    UtiliLine::get_parallel_candidate(parali_id.second, opti_para.maplines, para_inlier, individual_line);
                    // sets of paraline and ids
                    if (para_inlier.size() > 1)
                    {
                        paralineid_mlids[vanishing_id] = para_inlier;
                        vanishing_id += 1;
                    }
                    // individule lines
                    for (int i = 0; i < individual_line.size(); i++)
                    {
                        single_line.push_back(individual_line[i]);
                        std::cout << "single:" << individual_line[i] << std::endl;
                    }
                }

                // 1. deal with paraline
                std::map<int /*vd_id*/, Eigen::Vector2d> paraline_orth_shared;                             // the shared parameters
                std::map<int /*vd_id*/, Eigen::Matrix<double, 3, 2>> paraline_orth_shared_tangent_vectors; // tangent_basis for shared parameters
                std::map<int /*vd_id*/, Vector6d> paraline_orth_shared_PLK;
                std::map<int /*mapline_id*/, Eigen::Vector2d> paraline_orth_unique;
                std::map<int /*vd_id*/, Eigen::Matrix<double, 6, 1>> paraline_plk_anchors;
                for (auto &parali_id : paralineid_mlids)
                {
                    std::cout << "******checking: vanishing_id:" << parali_id.first << std::endl;
                    // 1.1 the shared part
                    int vanishing_direction_id = parali_id.first;
                    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
                    // anchor_PLK
                    Eigen::Matrix<double, 6, 1> LinePLK0 = Eigen::Matrix<double, 6, 1>::Zero(); // Zero();
                    for (auto &ml_id : parali_id.second)
                    {
                        Eigen::Vector3d StartPoint = opti_para.maplines[ml_id].col(0);
                        Eigen::Vector3d EndPoint = opti_para.maplines[ml_id].col(1);
                        Eigen::Vector3d LineDirection = EndPoint - StartPoint;
                        Eigen::Vector3d LinePlaneNormal = StartPoint.cross(EndPoint);
                        double d = LinePlaneNormal.norm() / LineDirection.norm();
                        LinePlaneNormal = LinePlaneNormal.normalized();
                        LineDirection = LineDirection.normalized();
                        std::cout << "direction:" << LineDirection << std::endl;
                        LinePLK0 << d * LinePlaneNormal, LineDirection;
                        direction += LineDirection;
                    }
                    // avarage direction vector
                    direction /= parali_id.second.size();
                    direction.normalize();
                    // initialize LinePLK0
                    Vec3 planeNormal = LinePLK0.head(3) - direction.head(3) * (direction.transpose() * LinePLK0.head(3));
                    LinePLK0.head(3) = planeNormal; // LinePLK0.head(3).norm() * planeNormal;
                    LinePLK0.tail(3) = direction;
                    // initialize TangentBasisVectors for LinePLK0
                    Eigen::Matrix<double, 3, 2> TangentBasisVectors = UtiliLine::TangentBasis(LinePLK0.tail(3)); // direction);
                    paraline_orth_shared_tangent_vectors[vanishing_direction_id] = TangentBasisVectors;
                    paraline_orth_shared[vanishing_direction_id] = Eigen::Vector2d(0, 0);
                    paraline_orth_shared_PLK[vanishing_direction_id] = LinePLK0;
                    problem.AddParameterBlock(paraline_orth_shared[vanishing_direction_id].data(), 2);

                    std::cout << "****original shared:" << vanishing_direction_id << std::endl;

                    // 1.2 the unique part
                    for (auto &ml_id : parali_id.second)
                    {
                        Eigen::Vector3d StartPoint = opti_para.maplines[ml_id].col(0);
                        Eigen::Vector3d EndPoint = opti_para.maplines[ml_id].col(1);
                        Eigen::Vector3d LineDirection = EndPoint - StartPoint;
                        Eigen::Vector3d LinePlaneNormal = StartPoint.cross(EndPoint);
                        double d = LinePlaneNormal.norm() / LineDirection.norm();
                        LinePlaneNormal = LinePlaneNormal.normalized();
                        LineDirection = LineDirection.normalized();
                        Eigen::Matrix<double, 6, 1> LinePLK1 = Eigen::Matrix<double, 6, 1>::Zero();
                        LinePLK1 << d * LinePlaneNormal, LineDirection;
                        Eigen::Vector4d para_line = UtiliLine::PLKPLK2ParaLine(LinePLK0, LinePLK1);
                        paraline_orth_unique[ml_id] = Eigen::Vector2d(para_line(2), para_line(3));

                        std::cout << "ml_id:" << ml_id << "," << para_line(2) << "," << para_line(3) << std::endl;

#if __DEBUG__OFF
                        Vector6d PLK1 = UtiliLine::ParaLine2PLK(LinePLK0, para_line, TangentBasisVectors);
                        std::cout << "======original line=======" << std::endl;
                        std::cout << "LinePLK1:" << LinePLK1.transpose() << std::endl;
                        std::cout << "distance:" << LinePLK1.head(3).norm() / LinePLK1.tail(3).norm() << std::endl;
                        std::cout << "======recovered line=======" << std::endl;
                        std::cout << "PLK1:" << PLK1.transpose() << std::endl;
                        std::cout << "distance:" << PLK1.head(3).norm() / PLK1.tail(3).norm() << std::endl;
#endif
                        problem.AddParameterBlock(paraline_orth_unique[ml_id].data(), 2); //, parali_parameterization);
                        // ceres::LocalParameterization *parali_parameterization = new ParaLineOrthParameterization();
                        // problem.AddParameterBlock(paraline_orth_unique[ml_id].data(), 2, parali_parameterization);
                    }
                }

                // 2. deal with unparallel_line
                double Single_Line_Feature[single_line.size()][4];
                for (auto ml = opti_para.maplines.begin(); ml != opti_para.maplines.end(); ml++)
                {
                    int ml_id = ml->first;
                    if (opti_para.maplines[ml_id] == Mat32::Zero())
                        continue;

                    bool skip = true;
                    // find unparallel_line
                    int single_line_idx = -1;
                    for (int k = 0; k < single_line.size(); k++)
                    {
                        if (ml_id == single_line[k])
                        {
                            skip = false;
                            // ml_id = k;
                            // break;
                            single_line_idx = k;
                        }
                    }
                    if (skip)
                        continue;

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
#if __DEBUG__OFF
                    // check plk_to_orth  and  orth_to_plk
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
                    Single_Line_Feature[single_line_idx][0] = LineOrth(0);
                    Single_Line_Feature[single_line_idx][1] = LineOrth(1);
                    Single_Line_Feature[single_line_idx][2] = LineOrth(2);
                    Single_Line_Feature[single_line_idx][3] = LineOrth(3);
                    // parameterization for orth
                    ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
                    problem.AddParameterBlock(Single_Line_Feature[single_line_idx], 4, local_parameterization_line); // p,q
                }

                //----> camera_pose
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

                    problem.AddParameterBlock(Para_Pose[kf_id], 7, pose_parameterization);
                    if (kf_id == 0)
                        problem.SetParameterBlockConstant(Para_Pose[kf_id]);
                }

                //--->  construct ceres problem
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

                for (auto asso_mpl_mea = paralineid_mlids.begin();
                     asso_mpl_mea != paralineid_mlids.end(); asso_mpl_mea++)
                {
                    int mpl_id = asso_mpl_mea->first; // map_paraline_id

                    // if (opti_para.maplines[mpl_id] == Mat32::Zero())
                    //     continue;
                    if (asso_mpl_mea->second.size() < 2)
                        continue;
                    for (int i = 0; i < asso_mpl_mea->second.size(); i++)
                    {
                        int ml_id = asso_mpl_mea->second.at(i); // map_line_id
                                                                // asso_mpl_mea->second[i].
                        if (opti_para.maplines[ml_id] == Mat32::Zero())
                            continue;

                        for (auto &ml_meas : opti_para.asso_ml_meas[ml_id])
                        {
                            int frame_id_i = ml_meas.first;
                            Eigen::Vector4d ml_obser_i = ml_meas.second;
                            Eigen::Vector4d mp_obser_i_unit(
                                (ml_obser_i(0) - cx) / fx,
                                (ml_obser_i(1) - cy) / fy,
                                (ml_obser_i(2) - cx) / fx,
                                (ml_obser_i(3) - cy) / fy);

                            // const Eigen::Matrix<double, 2, 2> sqrt_information = Eigen::Matrix<double, 2, 2>::Identity();
                            Vector6d PLK0 = paraline_orth_shared_PLK[mpl_id];
                            Eigen::Matrix<double, 3, 2> TangentBasisVectors = paraline_orth_shared_tangent_vectors[mpl_id];

                            ParaLineProjectionFactor *cost_function = new ParaLineProjectionFactor(mp_obser_i_unit, PLK0, TangentBasisVectors); // \u7279\u5f81\u91cd\u6295\u5f71\u8bef\u5dee
                            problem.AddResidualBlock(cost_function, loss_function,
                                                     Para_Pose[frame_id_i],
                                                     paraline_orth_shared[mpl_id].data(),
                                                     paraline_orth_unique[ml_id].data());
                        }

                        // for (auto &ml_meas :
                        //      opti_para.asso_ml_posi[ml_id]) // get 2D features associated
                        //                                     // to this mapline
                        // {
                        //     int frame_id = ml_meas.first;
                        //     auto uvd = opti_para.maplines[ml_id];
                        //     Eigen::Vector2d enddepths;
                        //     enddepths << uvd(2, 0), uvd(2, 1);
                        //     // uv pixel
                        //     Eigen::Matrix<double, 6, 1> endpixel_d_s = ml_meas.second;

                        //     Eigen::Matrix<double, 4, 1> mp_obser_i_unit;
                        //     mp_obser_i_unit << (endpixel_d_s(0, 0) - cx) / fx,
                        //         (endpixel_d_s(1, 0) - cy) / fy,
                        //         (endpixel_d_s(3, 0) - cx) / fx,
                        //         (endpixel_d_s(4, 0) - cy) / fy;

                        //     const Eigen::Matrix<double, 2, 2> sqrt_information = Eigen::Matrix<double, 2, 2>::Identity();
                        //     Vector6d PLK0 = paraline_orth_shared_PLK[mpl_id];
                        //     Eigen::Matrix<double, 3, 2> TangentBasisVectors = paraline_orth_shared_tangent_vectors[mpl_id];

                        //     ParaLineProjectionFactor *cost_function = new ParaLineProjectionFactor(mp_obser_i_unit, PLK0, TangentBasisVectors); // \u7279\u5f81\u91cd\u6295\u5f71\u8bef\u5dee
                        //     problem.AddResidualBlock(cost_function, loss_function,
                        //                              Para_Pose[frame_id],
                        //                              paraline_orth_shared[mpl_id].data(),
                        //                              paraline_orth_unique[ml_id].data());

                        //     // ParaLineRotationFactor *rot_cost_function = new ParaLineRotationFactor(mp_obser_i_unit, PLK0, TangentBasisVectors);
                        //     // problem.AddResidualBlock(rot_cost_function, loss_function,
                        //     //                         Para_Pose[frame_id], paraline_orth_shared[mpl_id].data());
                        // }
                    }
                }

                for (auto asso_ml_mea = opti_para.asso_ml_meas.begin(); asso_ml_mea != opti_para.asso_ml_meas.end(); asso_ml_mea++)
                {
                    int ml_id = asso_ml_mea->first;
                    if (opti_para.maplines[ml_id] == Mat32::Zero())
                        continue;
                    bool skip = true;
                    int single_line_idx = -1;
                    for (int k = 0; k < single_line.size(); k++)
                    {
                        if (ml_id == single_line[k])
                        {
                            skip = false;
                            // ml_id = k;
                            single_line_idx = k;
                        }
                    }
                    if (skip)
                        continue;
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
                        problem.AddResidualBlock(f, loss_function, Para_Pose[kf_id_i], Single_Line_Feature[single_line_idx]);
                    }
                }

                // solve ceres problem
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.minimizer_progress_to_stdout = true;
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
                // std::map<int /*vd_id*/, Eigen::Vector2d> paraline_orth_shared;
                // std::map<int /*mapline_id*/, Eigen::Vector2d> paraline_orth_unique;
                for (auto &para_shared : paraline_orth_shared)
                {
                    int vd_id = para_shared.first;
                    Eigen::Vector2d shared_data = para_shared.second;
                    Vector6d PLK0 = paraline_orth_shared_PLK[vd_id];
                    std::cout << "*****shared: id" << vd_id << std::endl;
                    Eigen::Matrix<double, 3, 2> TangentBasisVectors = paraline_orth_shared_tangent_vectors[vd_id];
                    for (auto &ml_id : paralineid_mlids[vd_id])
                    {
                        Eigen::Vector2d unique_data = paraline_orth_unique[ml_id];
                        //  from shared_data and unique_data to endpoints
                        Vector4d parali(shared_data(0), shared_data(1), unique_data(0), unique_data(1));

                        std::cout << "ml_id" << ml_id << "," << shared_data(0) << "," << shared_data(1) << "," << unique_data(0) << "," << unique_data(1) << std::endl;

                        Vector6d PLK1 = UtiliLine::ParaLine2PLK(PLK0, parali, TangentBasisVectors);
                        Vector6d EndPoints3d;
                        EndPoints3d << 0, 0, 0, 0, 0, 0;
                        int obser_num = 0;

                        for (int i = 0; i < opti_para.asso_ml_meas[ml_id].size(); i++)
                        {
                            Eigen::Vector4d ml_obser = opti_para.asso_ml_meas[ml_id][i].second;
                            Eigen::Vector4d mp_obser_i_unit(
                                (ml_obser(0) - cx) / fx,
                                (ml_obser(1) - cy) / fy,
                                (ml_obser(2) - cx) / fx,
                                (ml_obser(3) - cy) / fy);
                            int kf_id_i = opti_para.asso_ml_meas[ml_id][i].first;
                            Eigen::Matrix4d Twc = opti_para.kfs[kf_id_i];
                            Vector6d EndPoints3d_i = UtiliLine::plk_to_endpoints(PLK1, mp_obser_i_unit, Twc);
                            Eigen::Vector3d EndPoints3d_i_s_distance = EndPoints3d_i.head(3) - EndPoints3d.head(3);
                            Eigen::Vector3d EndPoints3d_i_e_distance = EndPoints3d_i.tail(3) - EndPoints3d.head(3);
                            double line_length = (EndPoints3d_i.head(3) - EndPoints3d_i.tail(3)).norm();
                            if (line_length < 0.1)
                                continue;
                            obser_num++;
                            if (EndPoints3d_i_s_distance.norm() < EndPoints3d_i_e_distance.norm())
                            {
                                EndPoints3d += EndPoints3d_i;
                            }
                            else
                            {
                                EndPoints3d.head(3) = EndPoints3d.head(3) + EndPoints3d_i.tail(3);
                                EndPoints3d.tail(3) = EndPoints3d.tail(3) + EndPoints3d_i.head(3);
                            }
                        }
                        EndPoints3d = EndPoints3d / obser_num;
                        optimized_maplines[ml_id].block(0, 0, 3, 1) = EndPoints3d.head(3);
                        optimized_maplines[ml_id].block(0, 1, 3, 1) = EndPoints3d.tail(3);
                    }
                }

                // update maplines
                for (auto asso_ml_mea = opti_para.asso_ml_meas.begin(); asso_ml_mea != opti_para.asso_ml_meas.end(); asso_ml_mea++)
                {
                    int ml_id = asso_ml_mea->first;

                    bool skip = true;
                    int single_line_idx = -1;
                    for (int k = 0; k < single_line.size(); k++)
                    {
                        if (ml_id == single_line[k])
                        {
                            skip = false;
                            // ml_id = k;
                            single_line_idx = k;
                        }
                    }
                    if (skip)
                        continue;

                    if (asso_ml_mea->second.size() < min_obser_num) // more than two frames detect this point
                        continue;

                    Eigen::Vector4d optimized_mapline_orth;
                    optimized_mapline_orth << Single_Line_Feature[single_line_idx][0],
                        Single_Line_Feature[single_line_idx][1],
                        Single_Line_Feature[single_line_idx][2],
                        Single_Line_Feature[single_line_idx][3];

                    Eigen::Matrix<double, 6, 1> opti_ml_plk = UtiliLine::orth_to_plk(optimized_mapline_orth);

                    Vector6d EndPoints3d;
                    EndPoints3d << 0, 0, 0, 0, 0, 0;
                    int obser_num = 0;

                    for (int i = 0; i < asso_ml_mea->second.size(); i++)
                    {
                        Eigen::Vector4d ml_obser = asso_ml_mea->second[i].second;
                        Eigen::Vector4d mp_obser_i_unit(
                            (ml_obser(0) - cx) / fx,
                            (ml_obser(1) - cy) / fy,
                            (ml_obser(2) - cx) / fx,
                            (ml_obser(3) - cy) / fy);

                        int kf_id_i = asso_ml_mea->second[i].first;
                        Eigen::Matrix4d Twc = opti_para.kfs[kf_id_i];

                        Vector6d EndPoints3d_i = UtiliLine::plk_to_endpoints(opti_ml_plk, mp_obser_i_unit, Twc);
                        Eigen::Vector3d EndPoints3d_i_s_distance = EndPoints3d_i.head(3) - EndPoints3d.head(3);
                        Eigen::Vector3d EndPoints3d_i_e_distance = EndPoints3d_i.tail(3) - EndPoints3d.head(3);
                        double line_length = (EndPoints3d_i.head(3) - EndPoints3d_i.tail(3)).norm();
                        if (line_length < 0.1)
                            continue;
                        obser_num++;
                        if (EndPoints3d_i_s_distance.norm() < EndPoints3d_i_e_distance.norm())
                        {
                            EndPoints3d += EndPoints3d_i;
                        }
                        else
                        {
                            EndPoints3d.head(3) = EndPoints3d.head(3) + EndPoints3d_i.tail(3);
                            EndPoints3d.tail(3) = EndPoints3d.tail(3) + EndPoints3d_i.head(3);
                        }
                    }
                    EndPoints3d = EndPoints3d / obser_num;

                    optimized_maplines[ml_id].block(0, 0, 3, 1) = EndPoints3d.head(3);
                    optimized_maplines[ml_id].block(0, 1, 3, 1) = EndPoints3d.tail(3);

                    std::cout << "original: " << opti_para.maplines[ml_id].block(0, 0, 3, 1).transpose() << "," << opti_para.maplines[ml_id].block(0, 1, 3, 1).transpose() << std::endl;
                    std::cout << "optimized: " << optimized_maplines[ml_id].block(0, 0, 3, 1).transpose() << "," << optimized_maplines[ml_id].block(0, 1, 3, 1).transpose() << std::endl;
                }

                std::cout << "optimized_maplines size: " << optimized_maplines.size() << std::endl;
                return;
            }

            // // find parallel lines from a coarse-parallel-line set
            // static int parallel_candidate(
            //     std::vector<int /*paramapline_id*/> &mpli_ids,
            //     std::map<int /*maplines_id*/, Eigen::Matrix<double, 3, 2>> &mplis,
            //     std::vector<int /*mapline_id*/> &para_inlier,
            //     std::vector<int /*mapline_id*/> &individure_outlier)
            // {
            //     int max_inlier = 0;
            //     for (int inter = 0; inter < 10; inter++)
            //     {
            //         int num = mpli_ids.size();
            //         int random_number = rand() % num; // generate a random integer between 0 and n
            //         Eigen::Vector3d startpoint_seed = mplis[mpli_ids[random_number]].col(0);
            //         Eigen::Vector3d endpoint_seed = mplis[mpli_ids[random_number]].col(1);
            //         Eigen::Vector3d line_direction_seed = (startpoint_seed - endpoint_seed).normalized();
            //         std::vector<int /*id*/> inliers;
            //         std::vector<int /*id*/> outliers;
            //         inliers.clear();
            //         outliers.clear();
            //         inliers.push_back(mpli_ids[random_number]);

            //         for (auto &ml_id : mpli_ids)
            //         {
            //             if (mplis[ml_id] == Mat32::Zero())
            //                 continue;

            //             Eigen::Vector3d startpoint = mplis[ml_id].col(0);
            //             Eigen::Vector3d endpoint = mplis[ml_id].col(1);
            //             Eigen::Vector3d line_direction = (startpoint - endpoint).normalized();
            //             // std::cout<<"seed:"<<random_number<<". "<<line_direction.transpose()*line_direction_seed<<","<<line_direction_seed.transpose()<<", "<<line_direction.transpose()<<std::endl;
            //             if (ml_id == mpli_ids[random_number])
            //                 continue;
            //             if (line_direction.transpose() * line_direction_seed > 0.90)
            //                 inliers.push_back(ml_id);
            //             else
            //                 outliers.push_back(ml_id);
            //         }

            //         if (inliers.size() > max_inlier)
            //         {
            //             max_inlier = inliers.size();
            //             para_inlier.clear();
            //             individure_outlier.clear();
            //             for (int j = 0; j < inliers.size(); j++)
            //                 para_inlier.push_back(inliers[j]);
            //             for (int j = 0; j < outliers.size(); j++)
            //                 individure_outlier.push_back(outliers[j]);
            //         }
            //     }
            // }
        };
    } // namespace optimizer
} // namespace simulator
#endif // __VENOM_SRC_OPTIMIZER_COVISEXTE_GRAPH_BUNDLEADJUSTMENT_2__
