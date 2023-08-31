/*
 * @Author: liyanyan liyanyan@meta-bounds.com
 * @Date: 2023-02-23 04:00:24
 * @LastEditors: liyanyan liyanyan@meta-bounds.com
 * @LastEditTime: 2023-02-23 04:05:05
 * @FilePath: /ParaLine-feature-refactor/src/optimizer/factor/PoseGraphSO3Factor.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */


#ifndef METABOUNDS_VIO_CAMERA_ONLY_FACTOR_HPP
#define METABOUNDS_VIO_CAMERA_ONLY_FACTOR_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>
//#include <sophus/so3.hpp>
#include "src/utils/IOFuntion.hpp"

namespace simulator {

    namespace Optimizer {

        class PoseGraphSE3Factor {
            public:
                PoseGraphSE3Factor(IO::Pose3d t_ab_measured,
                                Eigen::Matrix<double,6,6> sqrt_information)
                    : t_ab_measured_(std::move(t_ab_measured)),
                      sqrt_information_(std::move(sqrt_information))            
                { }

                template <typename T>
                bool operator()(const T* const p_a_ptr,  const T* const q_a_ptr,
                                const T* const p_b_ptr,  const T* const q_b_ptr,
                                T* residuals_ptr) const
                {
                    Eigen::Map<const Eigen::Matrix<T,3,1>>  p_a(p_a_ptr);
                    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

                    Eigen::Map<const Eigen::Matrix<T,3,1>> p_b(p_b_ptr);
                    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

                    // Compute the relative T
                    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
                    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse* q_b;

                    // Represent the displacement       in the A frame
                    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse* (p_b - p_a);
                    
                    // Compute the error 
                    Eigen::Quaternion<T> delta_q = 
                        t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

                    Eigen::Map<Eigen::Matrix<T,6,1>> residuals(residuals_ptr);
                    residuals.template block<3,1>(0,0) = 
                        p_ab_estimated - t_ab_measured_.p.template cast<T>();
                    residuals.template block<3,1>(3,0) = T(2.0) *delta_q.vec();

                    // scale the residuals
                    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

                    return true;      
                }

                static ceres::CostFunction* Create(
                     const IO::Pose3d &t_ab_measured, 
                     const Eigen::Matrix<double,6,6>& sqrt_information){
                        return new ceres::AutoDiffCostFunction<PoseGraphSE3Factor, 6, 3, 4, 3, 4>(
                            new PoseGraphSE3Factor(t_ab_measured, sqrt_information));
                    }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            private:
                const IO::Pose3d t_ab_measured_;
                const Eigen::Matrix<double,6,6> sqrt_information_;    
        };
    }
}


#endif //__METABOUNDS_VIO_CAMERA_ONLY_FACTOR_HPP__

