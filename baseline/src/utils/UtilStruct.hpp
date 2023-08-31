/*
 * @Author: liyanyan liyanyan@meta-bounds.com
 * @Date: 2023-01-14 10:20:27
 * @LastEditors: liyanyan liyanyan@meta-bounds.com
 * @LastEditTime: 2023-01-14 13:35:13
 * @FilePath: /ParaLine-main/src/util/UtilStruct.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-11-11 14:46:00
 * @LastEditTime: 2023-07-29 16:31:35
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /1. RA-L2023/src/utils/UtilStruct.hpp
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
}

#endif // __VENOM_SRC_UTIL_UtilStruct_HPP__

