/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-02-08 02:11:05
 * @LastEditTime: 2023-02-26 02:39:01
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /VENOM/src/optimizer/factor/PoseGraphSO3Factor.hpp
 */

#ifndef METABOUNDS_VIO_ROTATION_ONLY_FACTOR_HPP
#define METABOUNDS_VIO_ROTATION_ONLY_FACTOR_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>
//#include <sophus/so3.hpp>

namespace simulator{

    namespace Optimizer {

        class PoseGraphSO3Factor {
            public:
                PoseGraphSO3Factor(Eigen::Quaterniond rot_ab_measured,
                                   Eigen::Matrix<double,3,3> sqrt_information)
                    : rot_ab_measured_(rot_ab_measured),
                      sqrt_information_(sqrt_information)            
                {}

                // build a plane from two directions
                //

                template <typename T>
                bool operator()(/*const T* const calibration,*/
                                const T* const q_a_ptr,
                                const T* const q_b_ptr, T* residuals_ptr) const
                {
                    //Eigen::Map<const Eigen::Quaternion<T>> calib(calibration);
                    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
                    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

                    // Compute the relative T
                    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
                    // Eigen::Quaternion<T> q_ab_estimated = 
                    //     calib*q_a.conjugate() * q_b*calib.conjugate();
                    
                    Eigen::Quaternion<T> q_ab_estimated = 
                        q_a.conjugate() * q_b;

                    // Compute the error 
                    Eigen::Quaternion<T> delta_q = 
                        rot_ab_measured_.template cast<T>() * q_ab_estimated.conjugate();

                    Eigen::Map<Eigen::Matrix<T,3,1>> residuals(residuals_ptr);
                 
                    // scale the residuals
                    residuals.template block<3,1>(0,0) = sqrt_information_ *delta_q.vec();
                    // scale the residuals
                    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

                    return true;      
                }

                static ceres::CostFunction* Create(
                    const Eigen::Quaterniond &rot_ab_measured, 
                    const Eigen::Matrix<double,3,3> sqrt_information){
                        return new ceres::AutoDiffCostFunction<PoseGraphSO3Factor,3, 4, 4>(
                            new PoseGraphSO3Factor(rot_ab_measured, sqrt_information));
                    }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            private:
                const Eigen::Quaterniond rot_ab_measured_;
                const Eigen::Matrix<double,3,3> sqrt_information_;    
        };
    }
    }


#endif //__METABOUNDS_VIO_ROTATION_ONLY_FACTOR_HPP__
