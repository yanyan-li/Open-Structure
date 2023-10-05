/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-09-30 06:55:26
 * @LastEditTime: 2023-10-05 03:23:24
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /baseline/src/utils/UtilStruct.hpp
 */

#ifndef __VENOM_SRC_UTIL_UtilStruct_HPP__
#define  __VENOM_SRC_UTIL_UtilStruct_HPP__

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

namespace simulator
{   
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Matrix<double, 3, 4> Mat34;
    typedef Eigen::Matrix<double, 3, 2> Mat32;
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Vector2d Vec2;
    typedef Eigen::Matrix4d Mat4;
    typedef Eigen::Matrix2d Mat2;
    typedef std::map<int/*frame_id*/, Vec3/*index_pixel*/> frame_point_meas;
    typedef std::map<int/*frame_id*/, Mat32/*index_pixel*/> frame_line_meas;
    typedef std::pair<int/*anchor_frame_id*/, Mat32/*plucker_in_anchor*/> ancframe_plucker;

    struct FrameFeatures
    {
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::map<int/*mappoint_id*/,  Vec2/*pixel_position*/> pixels;
        std::map<int/*mapline_id*/, Mat2/*endpoints_position*/> endpoints;
    };

    struct  EnvironmentParameter
    {
        int frame_num_;
        int traject_type_;
        int vert_lines_;
        int horiz_lines_;
        int vert_points_;
        int horiz_points_;
        double distance_ ; // distance between wall and the trajectory center
        bool add_noise_to_meas_;

        // size of the bbx
        double length_;
        double width_;
        double height_;
    };

    struct OptimizationManager
    {
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /* data*/
        std::map<int /*kf_id*/, Eigen::Matrix4d> kfs;
        std::map<int /*mappoint_id*/, Eigen::Vector3d> mappoints;
        std::map<int /*maplines_id*/, Eigen::Matrix<double, 3, 2>> maplines;
        /* association*/
        std::map<int /*mappoint_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>>> asso_mp_meas;
        std::map<int /*mapline_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Vector4d /*2d_meas*/>>> asso_ml_meas;
        std::map<int /*mapline_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Matrix<double,6,1> /*3d_posi*/>>> asso_ml_posi;
        std::map<int /*parali_id*/, std::vector<int /*mapline_id*/>> asso_paralineid_mlids; 
        std::map<int /*kf_id*/, std::vector<int /*mappoint_id*/>> mp_obsers;
        std::map<int /*kf_id*/, std::vector<int /*mapline_id*/>> ml_obsers;
    };

    double color_table[10][3] =
        {{0.1f, 0.1f, 0.0f},
         {0.3f, 0.3f, 0.5f},
         {0.6f, 0.5f, 1.0f},
         {0.9f, 0.7f, 0.9f},
         {1.0f, 0.9f, 0.7f},
         {0.1f, 0.7f, 0.5f},
         {0.3f, 0.5f, 0.7f},
         {0.6f, 0.3f, 0.3f},
         {0.9f, 0.5f, 0.7f},
         {0.1f, 0.8f, 0.6f}};
}

#endif // __VENOM_SRC_UTIL_UtilStruct_HPP__

