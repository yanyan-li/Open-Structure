/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-06 02:33:29
 * @LastEditTime: 2023-08-27 02:57:21
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /JointFactorGraph/src/utils/UtilEnvElement.hpp
 */
#ifndef __VENOM_SRC_UTIL_UTILENVELEMENT_HPP__
#define __VENOM_SRC_UTIL_UTILENVELEMENT_HPP__

#include <Eigen/StdVector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <vector>
class Cube
{
public:
    Cube(const Eigen::Vector3d origin_w,
         const double cube_l,
         const double cube_w,
         const double cube_h)
    {
        cube_origin_w_ = origin_w;
        cube_l_ = cube_l;
        cube_w_ = cube_w;
        cube_h_ = cube_h;
        cube_center_ << cube_l_ / 2, cube_w_ / 2, cube_h_ / 2;

        SetCubeVerticesWorld();
        SetCubePlanesWorld(plane_bottom_w_, cube_origin_w_, vertex_1_w_, vertex_2_w_, vertex_3_w_);
        SetCubePlanesWorld(plane_top_w_, vertex_4_w_, vertex_5_w_, vertex_6_w_, vertex_7_w_);
        SetCubePlanesWorld(plane_left_w_, vertex_4_w_, cube_origin_w_, vertex_3_w_, vertex_7_w_);
        SetCubePlanesWorld(plane_right_w_, vertex_5_w_, vertex_1_w_, vertex_2_w_, vertex_6_w_);
        SetCubePlanesWorld(plane_front_w_, vertex_7_w_, vertex_6_w_, vertex_2_w_, vertex_3_w_);
        SetCubePlanesWorld(plane_back_w_, vertex_4_w_, vertex_5_w_, vertex_1_w_, cube_origin_w_);
    }

    void SetCubeVerticesWorld()
    { //  x: w;  y:l;  z:h
        vertex_1_w_ << cube_origin_w_(0), cube_origin_w_(1) + cube_l_, cube_origin_w_(2);
        vertex_2_w_ << cube_origin_w_(0) + cube_w_, cube_origin_w_(1) + cube_l_, cube_origin_w_(2);
        vertex_3_w_ << cube_origin_w_(0) + cube_w_, cube_origin_w_(1), cube_origin_w_(2);

        vertex_4_w_ << cube_origin_w_(0), cube_origin_w_(1), cube_origin_w_(2) + cube_h_;
        vertex_5_w_ << cube_origin_w_(0), cube_origin_w_(1) + cube_l_, cube_origin_w_(2) + cube_h_;
        vertex_6_w_ << cube_origin_w_(0) + cube_w_, cube_origin_w_(1) + cube_l_, cube_origin_w_(2) + cube_h_;
        vertex_7_w_ << cube_origin_w_(0) + cube_w_, cube_origin_w_(1), cube_origin_w_(2) + cube_h_;
    }

    void SetCubePlanesWorld(std::vector<Eigen::Vector3d> &plane_vertices,
                            Eigen::Vector3d vertex0,
                            Eigen::Vector3d vertex1,
                            Eigen::Vector3d vertex2,
                            Eigen::Vector3d vertex3)
    {
        plane_vertices.push_back(vertex0);
        plane_vertices.push_back(vertex1);
        plane_vertices.push_back(vertex2);
        plane_vertices.push_back(vertex3);
    }

    void GetCubeVertices(std::map<int /*vertex_id*/, Eigen::Vector3d /*coord_w*/> &coordinates_w)
    {
        coordinates_w[0] = cube_origin_w_;
        coordinates_w[1] = vertex_1_w_;
        coordinates_w[2] = vertex_2_w_;
        coordinates_w[3] = vertex_3_w_;

        coordinates_w[4] = vertex_4_w_;
        coordinates_w[5] = vertex_5_w_;
        coordinates_w[6] = vertex_6_w_;
        coordinates_w[7] = vertex_7_w_;
    }

    void GetCubePlanes(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_planes, double theta = 0)
    {
        Eigen::Matrix3d Rwb;
        Rwb = Eigen::AngleAxisd(theta / 2, Eigen::Vector3d::UnitZ()); // * Eigen::AngleAxisd(3.1415 / 2., Eigen::Vector3d::UnitX());
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 0, 3, 3) = Rwb;

        auto Rot = [&](std::vector<Eigen::Vector3d> &plane_vertices)
        {
            for (size_t i = 0; i < plane_vertices.size(); i++)
                plane_vertices[i] = Rwb * plane_vertices[i];
        };
        Rot(plane_bottom_w_);
        cube_planes["bottom"] = plane_bottom_w_;
        Rot(plane_top_w_);
        cube_planes["top"] = plane_top_w_;
        Rot(plane_left_w_);
        cube_planes["left"] = plane_left_w_;
        Rot(plane_right_w_);
        cube_planes["right"] = plane_right_w_;
        Rot(plane_front_w_);
        cube_planes["front"] = plane_front_w_;
        Rot(plane_back_w_);
        cube_planes["back"] = plane_back_w_;
    }

    void GetCubeInfo(std::vector<double> &cube_info)
    {
        cube_info.push_back(cube_l_);
        cube_info.push_back(cube_w_);
        cube_info.push_back(cube_h_);
    }

private:
    double cube_l_;
    double cube_w_;
    double cube_h_;
    Eigen::Vector3d cube_center_;
    // vertices in clock-wise
    Eigen::Vector3d cube_origin_w_;
    Eigen::Vector3d vertex_1_w_;
    Eigen::Vector3d vertex_2_w_;
    Eigen::Vector3d vertex_3_w_;

    Eigen::Vector3d vertex_4_w_;
    Eigen::Vector3d vertex_5_w_;
    Eigen::Vector3d vertex_6_w_;
    Eigen::Vector3d vertex_7_w_;
    // planes in clock-wise
    std::vector<Eigen::Vector3d> plane_bottom_w_;
    std::vector<Eigen::Vector3d> plane_top_w_;
    std::vector<Eigen::Vector3d> plane_left_w_;
    std::vector<Eigen::Vector3d> plane_right_w_;
    std::vector<Eigen::Vector3d> plane_front_w_;
    std::vector<Eigen::Vector3d> plane_back_w_;
};

// class Plane
// {
// public:
//     Plane(const Eigen::Vector3d &left_top_w,
//           const Eigen::Vector3d &right_bottom_w)
//     {
//         vertex_left_top_w_ = left_top_w;
//         vertex_right_bottom_w_ = right_bottom_w;
//         vertex_right_top_w_ << right_bottom_w(0),
//             left_top_w
//     }
//     SetPlaneVertices(Eigen::Vector3d &left_top_w,
//                      const)

//         private : Eigen::Vector3d vertex_left_top_w_;
//     Eigen::Vector3d vertex_right_bottom_w_;
//     Eigen::Vector3d vertex_left_bottom_w_;
//     Eigen::Vector3d vertex_right_top_w_;
// };

// class Circle
// {
// public:
//     Circle(const Eigen::Vector3d center_w,
//            const Eigen::Vecter3d normal_w,
//            const double ridus_w)
//     {
//         double normal_length = normal_w.norm();

//         Eigen::Vector3d normal = normal_w.normalized();
//     }

// private:
//     Eigen::Vector3d center_w_;
//     Eigen::Vector3d normal_w_;
// }

class Corridor
{
public:
    Corridor(const double corr_l, const double corr_w, const double corr_h)
    {
        // top region
        Eigen::Vector3d cube_top_origin_w = Eigen::Vector3d(-0.5 * corr_w, -0.3 * corr_l, -0.5 * corr_h); // w, l h
        cube_top_ = new Cube(cube_top_origin_w, corr_l * 0.6, corr_w * 0.2, corr_h);

        // right region
        Eigen::Vector3d cube_right_origin_w = Eigen::Vector3d(-0.5 * corr_w, 0.3 * corr_l, -0.5 * corr_h); // w, l h
        cube_right_ = new Cube(cube_right_origin_w, corr_l * 0.2, corr_w, corr_h);

        // bottom region
        Eigen::Vector3d cube_bottom_origin_w = Eigen::Vector3d(0.3 * corr_w, -0.3 * corr_l, -0.5 * corr_h); // w, l h
        cube_bottom_ = new Cube(cube_bottom_origin_w, corr_l * 0.6, corr_w * 0.2, corr_h);

        // left region
        Eigen::Vector3d cube_left_origin_w = Eigen::Vector3d(-0.5 * corr_w, -0.5 * corr_l, -0.5 * corr_h); // w, l h
        cube_left_ = new Cube(cube_left_origin_w, corr_l * 0.2, corr_w, corr_h);
    }

    void GetRegions(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_top_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_right_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_bottom_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_left_planes)
    {
        cube_top_->GetCubePlanes(cube_top_planes);
        cube_right_->GetCubePlanes(cube_right_planes);
        cube_bottom_->GetCubePlanes(cube_bottom_planes);
        cube_left_->GetCubePlanes(cube_left_planes);
    }
    void GetRegions(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_top_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_right_planes)
    {
        cube_top_->GetCubePlanes(cube_top_planes);
        cube_right_->GetCubePlanes(cube_right_planes);
    }

private:
    Cube *cube_top_;
    Cube *cube_right_;
    Cube *cube_bottom_;
    Cube *cube_left_;
};

class Room
{
public:
    Room(const double corr_l, const double corr_w, const double corr_h)
    {
        //
        Eigen::Vector3d room_origin_w = Eigen::Vector3d(-0.5 * corr_w, -0.5 * corr_l, -0.5 * corr_h);
        room_cube_ = new Cube(room_origin_w, corr_l, corr_w, corr_h);

        Eigen::Vector3d picture_origin_w = Eigen::Vector3d(-0.5 * corr_w, -0.4 * corr_l, 0);
        room_picture_ = new Cube(picture_origin_w, 0.8 * corr_l, 0.1 * corr_w, 0.4 * corr_h);

        Eigen::Vector3d cabinet_origin_w = Eigen::Vector3d(0.2 * corr_w, -0.5 * corr_l, -0.5 * corr_h);
        room_cabinet_ = new Cube(cabinet_origin_w, 0.3 * corr_l, 0.3 * corr_w, 0.6 * corr_h);

        Eigen::Vector3d fridge_origin_w = Eigen::Vector3d(0.3 * corr_w, 0.3 * corr_l, -0.5 * corr_h);
        room_fridge_ = new Cube(fridge_origin_w, 0.2 * corr_l, 0.2 * corr_w, 0.4 * corr_h);
    }

    void GetRegions(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &picture_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cabinet_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &fridge_planes)
    {
        // cube_top_->GetCubePlanes(cube_top_planes);
        room_cube_->GetCubePlanes(cube_planes);
        room_picture_->GetCubePlanes(picture_planes);
        room_cabinet_->GetCubePlanes(cabinet_planes);
        room_fridge_->GetCubePlanes(fridge_planes);
    }

private:
    // Cube *cube_top_;
    Cube *room_cube_;
    Cube *room_picture_;
    Cube *room_cabinet_;
    Cube *room_fridge_;
};

class Objects
{
public:
    Objects(const double corr_l, const double corr_w, const double corr_h)
    {
        Eigen::Vector3d floor_origin_w = Eigen::Vector3d(-0.5 * corr_w, -0.5 * corr_l, -0.5 * corr_h);
        objects_floor_ = new Cube(floor_origin_w, corr_l, corr_w, 0.1 * corr_h);

        Eigen::Vector3d box_1_origin_w = Eigen::Vector3d(-0.3 * corr_w, -0.3 * corr_l, -0.4 * corr_h);
        objects_box_1_ = new Cube(box_1_origin_w, 0.3 * corr_l, 0.3 * corr_w, 0.3 * corr_h);

        Eigen::Vector3d box_2_origin_w = Eigen::Vector3d(-0.3 * corr_w, 0.1 * corr_l, -0.4 * corr_h);
        objects_box_2_ = new Cube(box_2_origin_w, 0.3 * corr_l, 0.3 * corr_w, 0.3 * corr_h);

        Eigen::Vector3d box_3_origin_w = Eigen::Vector3d(0.1 * corr_w, 0 * corr_l, -0.4 * corr_h);
        objects_box_3_ = new Cube(box_3_origin_w, 0.3 * corr_l, 0.3 * corr_w, 0.3 * corr_h);

        Eigen::Vector3d box_4_origin_w = Eigen::Vector3d(-0.15 * corr_w, -0.15 * corr_l, -0.1 * corr_h);
        objects_box_4_ = new Cube(box_4_origin_w, 0.3 * corr_l, 0.3 * corr_w, 0.3 * corr_h);
    }

    void GetRegions(std::map<std::string, std::vector<Eigen::Vector3d>> &floor_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &box_1_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &box_2_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &box_3_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &box_4_planes)
    {
        // cube_top_->GetCubePlanes(cube_top_planes);
        objects_floor_->GetCubePlanes(floor_planes);
        objects_box_1_->GetCubePlanes(box_1_planes);
        objects_box_2_->GetCubePlanes(box_2_planes, 0.2);
        objects_box_3_->GetCubePlanes(box_3_planes, 0.3);
        objects_box_4_->GetCubePlanes(box_4_planes, 0.4);
    }

private:
    // Cube *cube_top_;
    Cube *objects_floor_;
    Cube *objects_box_1_;
    Cube *objects_box_2_;
    Cube *objects_box_3_;
    Cube *objects_box_4_;
};

#endif //__VENOM_SRC_UTIL_UTILENVELEMENT_HPP__