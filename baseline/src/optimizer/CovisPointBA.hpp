/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-02-04 16:28:14
 * @LastEditTime: 2023-08-29 13:54:11
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /JointFactorGraph/src/optimizer/CovisPointBA.hpp
 */

#ifndef __VENOM_SRC_OPTIMIZER_COVIS_POINT_BUNDLEADJUSMENT_CERES_HPP__
#define __VENOM_SRC_OPTIMIZER_COVIS_POINT_BUNDLEADJUSMENT_CERES_HPP__

#include <ceres/ceres.h>
#include "src/manager_env/EnvTrajectory.hpp" 
#include "src/optimizer/parametrization/line_parameterization.hpp"
#include "src/optimizer/parametrization/pose_local_parameterization.hpp"
#include "src/utils/UtilTransformer.hpp"
#include "src/optimizer/factor/LineProjectionFactor.hpp"

namespace simulator
{
    namespace optimizer
    {
        class CovisPointBA
        {
            public:

            static void optimizer(
                simulator::Trajectory *ptr_robot_trajectory,
                simulator::OptimizationManager &opti_para,
                std::map<int, Eigen::Vector3d> &optimized_mappoints,
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
                double fx; double fy; double cx; double cy;
                fx = ptr_robot_trajectory->cam_intri.fx;
                fy = ptr_robot_trajectory->cam_intri.fy;
                cx = ptr_robot_trajectory->cam_intri.cx;
                cy = ptr_robot_trajectory->cam_intri.cy;
             
                // construct ceres optim parameters
                ceres::Problem problem;
                ceres::LossFunction *loss_function;
                ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
                loss_function = new ceres::CauchyLoss(1.0);

                // std::cout << "loss_function: " << opti_para.mappoints.size() << std::endl;
                // add mappoints
                double Para_Point_Feature[opti_para.mappoints.size()][3];
                for(auto mp = opti_para.mappoints.begin(); mp!= opti_para.mappoints.end(); mp++)
                {
                    int mp_id = mp->first;

                    if (opti_para.mappoints[mp_id] == Vec3::Zero())
                        continue;
                    double point_distance = opti_para.mappoints[mp_id].norm();
                    if (point_distance < 0.1)
                        assert(0 == 1);
                    Para_Point_Feature[mp_id][0] = mp->second(0);
                    Para_Point_Feature[mp_id][1] = mp->second(1);
                    Para_Point_Feature[mp_id][2] = mp->second(2);
                    // std::cout<<mp->second(0)<<std::endl;
                    problem.AddParameterBlock(Para_Point_Feature[mp_id], 3);
                    // ordering 
                }

                // std::cout << "points: " << opti_para.mappoints.size() << std::endl;

                // add camera pose
                double Para_Pose[opti_para.kfs.size()][7];
                for (auto kf = opti_para.kfs.begin(); kf != opti_para.kfs.end(); kf++)
                {
                    int kf_id = kf->first;
                    Eigen::Matrix4d Twc = kf->second;
                    Eigen::Matrix3d Rot = Twc.block(0,0,3,3);
                    Eigen::Vector3d Tran = Twc.block(0,3,3,1);
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
                    if(kf_id == 0)
                        problem.SetParameterBlockConstant(Para_Pose[kf_id]);
                }

                // std::cout << "poses: " << opti_para.kfs.size() << std::endl;

                // construct ceres problem
                for(auto asso_mp_mea = opti_para.asso_mp_meas.begin(); asso_mp_mea !=  opti_para.asso_mp_meas.end(); asso_mp_mea++)
                {
                    int mp_id = asso_mp_mea->first;
                    if(opti_para.mappoints[mp_id]==Vec3::Zero())
                        continue;
                    if(asso_mp_mea->second.size() < min_obser_num) // more than two frames detect this point
                        continue;

                    for(int i = 0; i < asso_mp_mea->second.size(); i++)
                    {
                        int kf_id_i = asso_mp_mea->second[i].first;
                        Eigen::Vector2d mp_obser_i = asso_mp_mea->second[i].second;
                        
                        Eigen::Vector2d mp_obser_i_unit((mp_obser_i(0)-cx) / fx, (mp_obser_i(1)-cy) / fy);

                        PointProjectionFactor *f = new PointProjectionFactor(mp_obser_i_unit); 
                        problem.AddResidualBlock(f, loss_function, Para_Pose[kf_id_i], Para_Point_Feature[mp_id]);
                    }
                }

                // std::cout << "mea: " << opti_para.asso_mp_meas.size() << std::endl;

                // slove ceres problem
                ceres::Solver::Options options;
                options.minimizer_progress_to_stdout = true;
                options.max_num_iterations = 5;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                ceres::Solver::Summary summary;
                ceres::Solve (options, &problem, & summary);
#if __VERBOSE__
                std::cout << summary.FullReport() << std::endl;
#endif
                // update ceres results
                for (auto kf =  opti_para.kfs.begin(); kf !=  opti_para.kfs.end(); kf++)
                {
                    int kf_id = kf->first;
                    Eigen::Vector3d Tran(Para_Pose[kf_id][0], Para_Pose[kf_id][1], Para_Pose[kf_id][2]);
                    Eigen::Quaterniond qua(Para_Pose[kf_id][6], Para_Pose[kf_id][3], Para_Pose[kf_id][4], Para_Pose[kf_id][5]);
                    Eigen::Matrix3d Rot = qua.toRotationMatrix();
                    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                    Twc.block(0,0,3,3) = Rot;
                    Twc.block(0,3,3,1) = Tran;
                    // std::cout << "pose: " << kf->second << std::endl
                    //           << Twc << std::endl;
                    // kfs[kf_id] = Twc;
                    optimized_poses[kf_id]=Twc;
                }

                // update mappoints
                for (auto mp = opti_para.mappoints.begin(); mp != opti_para.mappoints.end(); mp++)
                {
                    int mp_id = mp->first;
                    if(opti_para.mappoints[mp_id]==Vec3::Zero())
                        continue;
                    Vec3 optimized_mappoint;
                    optimized_mappoint << Para_Point_Feature[mp_id][0], //= mp->second(0);
                        Para_Point_Feature[mp_id][1],                   //= mp->second(1);
                        Para_Point_Feature[mp_id][2];                   //= mp->second(2);
                    optimized_mappoints[mp_id] = optimized_mappoint;
                    // std::cout << "original: " << mp->second(0) << "," << mp->second(1) << "," << mp->second(2) << std::endl;
                    // std::cout << "optimized: " << optimized_mappoint(0) << "," << optimized_mappoint(1) << "," << optimized_mappoint(2) << std::endl;
                }
                return;
            }
        };
    } // namespace optimizer
} // namespace simulator

#endif // __VENOM_SRC_OPTIMIZER_COVIS_POINT_BUNDLEADJUSMENT_CERES_HPP__
