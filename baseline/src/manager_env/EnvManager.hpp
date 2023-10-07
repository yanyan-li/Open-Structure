/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-11-26 02:27:27
 * @LastEditTime: 2023-01-27 01:41:04
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: EnvManager is used to build the all elements, including trajectory and landmarks.
 * @FilePath: /VENOM/src/manager/EnvManager.hpp
 */
#ifndef __VENOM_SRC_MANAGER_EnvManager_HPP__
#define __VENOM_SRC_MANAGER_EnvManager_HPP__

#include "src/estimator/Track.hpp"
#include "src/manager_env/EnvFrame.hpp"
#include "src/manager_env/EnvLine.hpp"
#include "src/manager_env/EnvPoint.hpp"
#include "src/manager_env/EnvTrajectory.hpp"
#include "src/utils/IOFuntion.hpp"
#include "src/utils/UtilEnvElement.hpp"
#include "src/utils/UtilStruct.hpp"
#include <Eigen/StdVector>
#include <Eigen/src/Core/Matrix.h>

namespace simulator
{
    class Frame;
    class EnvManager
    {
    public:
        //------>
        simulator::Trajectory *ptr_robot_trajectory_; //
        simulator::Track *ptr_tracker_;               //
        // trajectory of the robot
        // std::map<int, Mat4> frameid_Twc_;
        std::map<int, Mat4, std::less<int>,
                 Eigen::aligned_allocator<std::pair<int, Mat4>>>
            frameid_Twc_;

        //------> association
        // get observations in each frame from envpoint_index
        std::map<int /*envpoint_id*/, frame_point_meas /*uvd*/> asso_epid_frameid_pixeld_;
        std::map<int /*envpoint_id*/, frame_point_meas /*xyz*/> asso_epid_frameid_pos_;
        std::map<int /*frame_id*/, std::vector<std::pair<int /*ep_id*/, Vec3>>> asso_frameid_epid_; // noisy measurement
        // get observations in each frame from envline_index
        std::map<int /*envline_id*/, frame_line_meas /*uvd*/> asso_elid_frameid_pixeld_; // XIN: used for optimization
        std::map<int /*envline_id*/, frame_line_meas /*xyz*/> asso_elid_frameid_pos_;    // TODO:
        std::map<int /*envline_id*/, ancframe_plucker /*ancframe_id_plucker*/> asso_elid_ancframeid_plucker_;
        std::map<int /*frame_id*/, std::vector<std::pair<int /*el_id*/, Mat32>>> asso_frameid_elid_; // noisy measurement
        // get parallel relationships
        std::map<int /*envparaline_id*/, std::vector<int /*envline_id*/>> asso_paralineid_elids_;

        // Frames
        // std::vector<Frame *> vec_ptr_frames;

    private:
        //------> parameters
        simulator::EnvironmentParameter env_para_;
        std::vector<Vec3> vec_points_;
        std::vector<Mat32> vec_lines_;
        std::vector<int> vec_lines_structlabel_;

        //------> map manager
        std::vector<std::pair<int, Vec3>> vec_epid_pos_w_;
        std::vector<std::pair<int, Mat32>> vec_elid_pos_w_;
        std::vector<std::pair<int /*vanishing_direction_id*/, std::vector<int /*el_id*/>>> vec_vdid_elids_;

        // settings
        bool b_add_noise_to_meas;
        double traj_radius;
        double env_width;
        double env_height;
        double env_length;

        //
        bool is_public;
        std::string traj_path;
        std::string envpoint_path_;
        std::string envline_path_;
        std::string associate_path_;
        std::string envline_associate_path_;
        int traj_num;
        int envpoint_num;
        int envline_num;

    public:
        EnvManager(simulator::EnvironmentParameter &env_para, cv::FileStorage &settings)
        {
            env_para_ = env_para;

            int frame_num = env_para.frame_num_;
            int traject_type = env_para.traject_type_;
            int vert_lines = env_para.vert_lines_;
            int horiz_lines = env_para.horiz_lines_;
            int vert_points = env_para.vert_points_;
            int horiz_points = env_para.horiz_points_;
            int env_points = vert_points + horiz_points;
            int env_lines = vert_lines + horiz_lines;

            settings["Track.noise_input"] >> b_add_noise_to_meas;
            settings["Env.Trajectory.radius"] >> traj_radius;
            settings["Env.Map.env_width"] >> env_width;
            settings["Env.Map.env_height"] >> env_height;
            settings["Env.Map.env_length"] >> env_length;

            settings["Env.Public.bool"] >> is_public;
            settings["Env.Public.envtraj_path"] >> traj_path;
            settings["Env.Public.envpoint_path"] >> envpoint_path_;
            settings["Env.Public.envline_path"] >> envline_path_;
            settings["Env.Public.associate_path"] >> associate_path_;

            // set cameras
            ptr_robot_trajectory_ = new simulator::Trajectory(traject_type, frame_num, settings);

            if (traject_type == 0)
                ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::CYCLE, frame_num, traj_radius);
            else if (traject_type == 1)
                ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::SPHERE, frame_num, traj_radius);
            else if (traject_type == 2)
                ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::CORRIDOR, frame_num, traj_radius);
            else if (traject_type == 9)
                ptr_robot_trajectory_->GenerateTrajectory(simulator::Trajectory::PUBLIC, frame_num, traj_radius);

            // init
            // vec_epid_pos_w_ = std::vector</*env_point_id*/ std::pair<int/*frame_id*/, Vec3/*posi_cam*/>>(vert_points, std::pair<int, Vec3>());
            // vec_elid_pos_w_ = std::vector</*env_line_id*/  std::pair<int/*frame_id*/, Mat32/*posi_cam*/>>(env_lines, std::pair<int, Mat32>());
        }

        void BuildEnvLines()
        {
            if (env_para_.traject_type_ == 2)
            {
                BuildCorridorLines();
                return;
            }
            else if (env_para_.traject_type_ == 0)
            {
                BuildCircleLines();
                return;
            }
            else if (env_para_.traject_type_ == 1)
            {
                BuildSphereLines();
                return;
            }
        }

        void BuildCirclePoints()
        {
            // get all walls
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;
            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->box_4_planes_)
            {
                planes.push_back(plane.second);
            }

            // generate mappoints based on walls
            for (int id = 0; id < env_para_.vert_points_; id++)
            {
                simulator::EnvPoint *ptr_ep = new simulator::EnvPoint(id, ptr_robot_trajectory_); // MapPoint(id, ptr_robot_trajectory_);

                int i = id % planes.size();

                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                ptr_ep->GenerateEnvPoint(plane_max_bounds, plane_min_bounds);

                assert(ptr_ep->pos_world_[0] != -10000); // haven't been initialized

                ptr_ep->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);
                // if (ptr_ep->obs_frame_pos_.size() < 3)
                //     std::cout << ">>>>> mp_id:" << id << std::endl;
                for (auto ob = ptr_ep->obs_frame_pos_.begin(), ob_end = ptr_ep->obs_frame_pos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Vec3 pos_in_cam = ob->second;
                    if (ptr_ep->obs_frame_pos_.size() < 3)
                        std::cout << "this:" << frame_id << ", " << pos_in_cam.x() << ", " << pos_in_cam.y() << std::endl;
                    asso_frameid_epid_[frame_id].push_back(std::make_pair(ptr_ep->num_id_, pos_in_cam));
                }

                // association
                if (ptr_ep->obs_frame_pixel_.size() > 0)
                {
                    asso_epid_frameid_pixeld_[id] = ptr_ep->obs_frame_pixel_;
                    asso_epid_frameid_pos_[id] = ptr_ep->obs_frame_pos_;
                    // gt
                    vec_points_.push_back(ptr_ep->pos_world_);
                    vec_epid_pos_w_.push_back(std::make_pair(ptr_ep->num_id_, ptr_ep->pos_world_)); // after checking
                }
                else
                {
                    std::cout << "association problems:" << ptr_ep->obs_frame_pixel_.begin()->first << ptr_ep->obs_frame_pixel_.begin()->second << std::endl;
                    std::cout << "the env_point cannot be detected by any views" << std::endl;

                    continue;
                }
            }
        }

        void BuildSpherePoints()
        {
            // get all walls
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;
            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }

            // generate mappoints based on walls
            for (int id = 0; id < env_para_.vert_points_; id++)
            {
                simulator::EnvPoint *ptr_ep = new simulator::EnvPoint(id, ptr_robot_trajectory_); // MapPoint(id, ptr_robot_trajectory_);

                int i = id % planes.size();

                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                ptr_ep->GenerateEnvPoint(plane_max_bounds, plane_min_bounds);

                assert(ptr_ep->pos_world_[0] != -10000); // haven't been initialized

                ptr_ep->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);
                if (ptr_ep->obs_frame_pos_.size() < 3)
                {
                    // std::cout << ">>>>> mp_id:" << id << std::endl;
                    delete ptr_tracker_;

                    id--;

                    continue;
                }

                for (auto ob = ptr_ep->obs_frame_pos_.begin(), ob_end = ptr_ep->obs_frame_pos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Vec3 pos_in_cam = ob->second;
                    if (ptr_ep->obs_frame_pos_.size() < 3)
                        std::cout << "this:" << frame_id << ", " << pos_in_cam.x() << ", " << pos_in_cam.y() << std::endl;
                    asso_frameid_epid_[frame_id].push_back(std::make_pair(ptr_ep->num_id_, pos_in_cam));
                }

                // association
                if (ptr_ep->obs_frame_pixel_.size() > 0)
                {
                    asso_epid_frameid_pixeld_[id] = ptr_ep->obs_frame_pixel_;
                    asso_epid_frameid_pos_[id] = ptr_ep->obs_frame_pos_;
                }
                else
                {
                    std::cout << "association problems" << ptr_ep->obs_frame_pixel_.begin()->first << ptr_ep->obs_frame_pixel_.begin()->second << std::endl;
                    // return;
                }

                // gt
                vec_points_.push_back(ptr_ep->pos_world_);
                vec_epid_pos_w_.push_back(std::make_pair(ptr_ep->num_id_, ptr_ep->pos_world_)); // after checking
            }
        }

        void BuildCorridorPoints()
        {
            // get all walls
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;
            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                // if (plane.first == "left")
                //     continue;
                // if (plane.first == "right")
                //     continue;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                // if (plane.first == "left")
                //     continue;
                // if (plane.first == "right")
                //     continue;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }

            // generate mappoints based on walls
            for (int id = 0; id < env_para_.vert_points_; id++)
            {
                simulator::EnvPoint *ptr_ep = new simulator::EnvPoint(id, ptr_robot_trajectory_); // MapPoint(id, ptr_robot_trajectory_);

                int i = id % planes.size();

                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                ptr_ep->GenerateEnvPoint(plane_max_bounds, plane_min_bounds);

                assert(ptr_ep->pos_world_[0] != -10000); // haven't been initialized

                ptr_ep->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);
                // if (ptr_ep->obs_frame_pos_.size() < 3)
                //     std::cout << ">>>>> mp_id:" << id << std::endl;
                for (auto ob = ptr_ep->obs_frame_pos_.begin(), ob_end = ptr_ep->obs_frame_pos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Vec3 pos_in_cam = ob->second;
                    if (ptr_ep->obs_frame_pos_.size() < 3)
                        std::cout << "this:" << frame_id << ", " << pos_in_cam.x() << ", " << pos_in_cam.y() << std::endl;
                    asso_frameid_epid_[frame_id].push_back(std::make_pair(ptr_ep->num_id_, pos_in_cam));
                }

                // association
                if (ptr_ep->obs_frame_pixel_.size() > 0)
                {
                    asso_epid_frameid_pixeld_[id] = ptr_ep->obs_frame_pixel_;
                    asso_epid_frameid_pos_[id] = ptr_ep->obs_frame_pos_;
                }
                else
                {
                    std::cout << "association problems" << ptr_ep->obs_frame_pixel_.begin()->first << ptr_ep->obs_frame_pixel_.begin()->second << std::endl;
                    // return;
                    assert(1 == 0);
                }

                // gt
                vec_points_.push_back(ptr_ep->pos_world_);
                vec_epid_pos_w_.push_back(std::make_pair(ptr_ep->num_id_, ptr_ep->pos_world_)); // after checking
            }
        }

        void BuildCircleLines()
        {
            // get all planes
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;

            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->box_4_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }

            // generate points
            for (int id = 0; id < env_para_.vert_lines_ /*500 map points*/; id++)
            {
                int i = id % planes.size();
                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                // std::cout << "plane_max_min:" << plane_max_bounds << "," << plane_min_bounds << std::endl;

                // initialize env_line
                simulator::EnvLine *ptr_ml = new simulator::EnvLine(id, ptr_robot_trajectory_);
                ptr_ml->GenerateEnvLine(plane_max_bounds, plane_min_bounds);

                int set_id = -1;
                set_id = ptr_ml->vanishing_direction_type_;

                ptr_ml->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);

                if (ptr_ml->obs_frameid_linepos_.size() < 3)
                    std::cout << ">>> ml_id:" << id << std::endl;
                for (auto ob = ptr_ml->obs_frameid_linepos_.begin(), ob_end = ptr_ml->obs_frameid_linepos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Mat32 pos_in_cam = ob->second;
                    asso_frameid_elid_[frame_id].push_back(std::make_pair(id, pos_in_cam));
                }

                asso_paralineid_elids_[set_id].push_back(id);
                // association
                if (ptr_ml->obs_frameid_linepixel_.size() > 0)
                {
                    asso_elid_frameid_pos_[id] = ptr_ml->obs_frameid_linepos_;
                    asso_elid_frameid_pixeld_[id] = ptr_ml->obs_frameid_linepixel_;
                }
                else
                {
                    std::cout << "association problems" << ptr_ml->obs_frameid_linepixel_.begin()->first << "," << ptr_ml->obs_frameid_linepixel_.begin()->second << std::endl;

                    // ep->obs_frame_pixel_.begin()->second << std::endl;
                }

                // gt
                vec_lines_.push_back(ptr_ml->pos_world_);
                vec_lines_structlabel_.push_back(ptr_ml->vanishing_direction_type_);

                vec_elid_pos_w_.push_back(std::make_pair(ptr_ml->num_id_, ptr_ml->pos_world_));
            }
        }

        void BuildSphereLines()
        {
            // get all planes
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;

            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout << "plane_id:" << plane.first << std::endl;
                planes.push_back(plane.second);
            }

            // generate points
            for (int id = 0; id < env_para_.vert_lines_ /*500 map points*/; id++)
            {
                int i = id % planes.size();
                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                // std::cout << "plane_max_min:" << plane_max_bounds << "," << plane_min_bounds << std::endl;

                // initialize env_line
                simulator::EnvLine *ptr_ml = new simulator::EnvLine(id, ptr_robot_trajectory_);
                ptr_ml->GenerateEnvLine(plane_max_bounds, plane_min_bounds);

                int set_id = -1;
                set_id = ptr_ml->vanishing_direction_type_;

                ptr_ml->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);

                if (ptr_ml->obs_frameid_linepos_.size() < 3)
                {
                    // std::cout << ">>>>> mp_id:" << id << std::endl;
                    delete ptr_ml;
                    id--;
                    continue;
                }

                for (auto ob = ptr_ml->obs_frameid_linepos_.begin(), ob_end = ptr_ml->obs_frameid_linepos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Mat32 pos_in_cam = ob->second;
                    asso_frameid_elid_[frame_id].push_back(std::make_pair(id, pos_in_cam));
                }

                asso_paralineid_elids_[set_id].push_back(id);
                // association
                if (ptr_ml->obs_frameid_linepixel_.size() > 0)
                {
                    asso_elid_frameid_pos_[id] = ptr_ml->obs_frameid_linepos_;
                    asso_elid_frameid_pixeld_[id] = ptr_ml->obs_frameid_linepixel_;
                }
                else
                {
                    std::cout << "association problems" << ptr_ml->obs_frameid_linepixel_.begin()->first << "," << ptr_ml->obs_frameid_linepixel_.begin()->second << std::endl;

                    // ep->obs_frame_pixel_.begin()->second << std::endl;
                }

                // gt
                vec_lines_.push_back(ptr_ml->pos_world_);
                vec_lines_structlabel_.push_back(ptr_ml->vanishing_direction_type_);

                vec_elid_pos_w_.push_back(std::make_pair(ptr_ml->num_id_, ptr_ml->pos_world_));
            }
        }

        void BuildCorridorLines()
        {
            // get all planes
            std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;

            for (auto plane : ptr_robot_trajectory_->cube_top_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_left_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_bottom_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }
            for (auto plane : ptr_robot_trajectory_->cube_right_planes_)
            {
                // std::cout<<"plane_id:"<<plane.first<<std::endl;
                planes.push_back(plane.second);
            }

            // generate points
            for (int id = 0; id < env_para_.vert_lines_ /*500 map points*/; id++)
            {
                int i = id % planes.size();
                Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
                Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
                GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
                // std::cout << "plane_max_min:" << plane_max_bounds << "," << plane_min_bounds << std::endl;

                // initialize env_line
                simulator::EnvLine *ptr_ml = new simulator::EnvLine(id, ptr_robot_trajectory_);
                ptr_ml->GenerateEnvLine(plane_max_bounds, plane_min_bounds);

                int set_id = -1;
                set_id = ptr_ml->vanishing_direction_type_;

                ptr_ml->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, b_add_noise_to_meas);

                if (ptr_ml->obs_frameid_linepos_.size() < 3)
                    std::cout << ">>> ml_id:" << id << std::endl;
                for (auto ob = ptr_ml->obs_frameid_linepos_.begin(), ob_end = ptr_ml->obs_frameid_linepos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Mat32 pos_in_cam = ob->second;
                    asso_frameid_elid_[frame_id].push_back(std::make_pair(id, pos_in_cam));
                }

                asso_paralineid_elids_[set_id].push_back(id);
                // association
                if (ptr_ml->obs_frameid_linepixel_.size() > 0)
                {
                    asso_elid_frameid_pos_[id] = ptr_ml->obs_frameid_linepos_;
                    asso_elid_frameid_pixeld_[id] = ptr_ml->obs_frameid_linepixel_;
                }
                else
                {
                    std::cout << "association problems" << ptr_ml->obs_frameid_linepixel_.begin()->first << "," << ptr_ml->obs_frameid_linepixel_.begin()->second << std::endl;

                    // ep->obs_frame_pixel_.begin()->second << std::endl;
                }

                // gt
                vec_lines_.push_back(ptr_ml->pos_world_);
                vec_lines_structlabel_.push_back(ptr_ml->vanishing_direction_type_);

                vec_elid_pos_w_.push_back(std::make_pair(ptr_ml->num_id_, ptr_ml->pos_world_));
            }
        }

        void BuildEnvPoints()
        {
            if (env_para_.traject_type_ == 2)
            {
                BuildCorridorPoints();
                return;
            }
            else if (env_para_.traject_type_ == 0)
            {
                BuildCirclePoints();
                return;
            }
            else if (env_para_.traject_type_ == 1)
            {
                // TODO:
                BuildSpherePoints();
                return;
            }
        }

        void BuildPublicLines()
        {
            std::vector<Eigen::Matrix<double, 7, 1>> maplines;
            IO::ReadPublicLineClouds(envline_path_, maplines);
            std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>> asso_elid_frameid;
            IO::ReadPublicLineAssociation(associate_path_, asso_elid_frameid);
            std::vector<std::pair<int, std::vector<int>>> asso_vdid_lineids;
            IO::ReadPublicParalineAssociation(associate_path_, asso_vdid_lineids);

            for (int i = 0; i < maplines.size(); i++)
            {
                // set_id for parallel lines
                int set_id = -1;

                int id = maplines[i][0];

                simulator::EnvLine *ptr_ml = new simulator::EnvLine(id, ptr_robot_trajectory_);
                ptr_ml->GenerateEnvLine(maplines[i], false);
                ptr_ml->GenerateStructLabel(asso_vdid_lineids);

                set_id = ptr_ml->vanishing_direction_type_;

                // ptr_ml->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_,
                //                        b_add_noise_to_meas);

                ptr_ml->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, asso_elid_frameid,
                                       b_add_noise_to_meas);

                for (auto ob = ptr_ml->obs_frameid_linepos_.begin(), ob_end = ptr_ml->obs_frameid_linepos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Mat32 pos_in_cam = ob->second;
                    asso_frameid_elid_[frame_id].push_back(std::make_pair(id, pos_in_cam));
                }

                // association
                // assert(set_id>0);
                if (ptr_ml->obs_frameid_linepixel_.size() > 0)
                {
                    if (set_id >= 0)
                        asso_paralineid_elids_[set_id].push_back(id);
                    asso_elid_frameid_pos_[id] = ptr_ml->obs_frameid_linepos_;
                    asso_elid_frameid_pixeld_[id] = ptr_ml->obs_frameid_linepixel_;
                }

                // gt
                vec_lines_.push_back(ptr_ml->pos_world_);
                vec_lines_structlabel_.push_back(ptr_ml->vanishing_direction_type_);
                vec_elid_pos_w_.push_back(std::make_pair(ptr_ml->num_id_, ptr_ml->pos_world_));
            }
        }

        void BuildPublicParalines()
        {
            std::map<int, Eigen::Matrix<double, 6, 1>> maplines;
            IO::ReadPublicLineClouds(envline_path_, maplines);
            std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>> asso_elid_frameid;
            std::vector<std::pair<int, std::vector<int>>> asso_vdid_lineids;
            IO::ReadPublicLineAssociation(associate_path_, asso_elid_frameid);
            IO::ReadPublicParalineAssociation(associate_path_, asso_vdid_lineids);

            for (auto asso_i : asso_vdid_lineids)
            {
                int vd_id = asso_i.first;
                for (int i = 0; i < asso_i.second.size(); i++)
                {
                    Eigen::Matrix<double, 6, 1> ml = maplines[asso_i.second[i]];
                    Eigen::Vector3d direct = (ml.head(3) - ml.tail(3)).normalized();

                    for (int j = i + 1; j < asso_i.second.size(); j++)
                    {
                        Eigen::Matrix<double, 6, 1> ml_j = maplines[asso_i.second[j]];
                        Eigen::Vector3d direct_j = (ml_j.head(3) - ml_j.tail(3)).normalized();
                        double distance = direct.transpose() * direct_j;
                        std::cout << "angle comparison in ids: " << asso_i.second[i] << "," << asso_i.second[j] << std::endl;
                        if (abs(distance) < 0.98)
                            std::cout << "angle:" << distance << std::endl;
                    }
                }
            }
        }

        void BuildPublicPoints()
        {
            // read map_point
            std::vector<std::pair<int /*point_id*/, Eigen::Vector3d /*point_position*/>> pts;
            IO::ReadPublicPointClouds(envpoint_path_, pts);
            // read observation
            std::vector<std::pair<int /*mp_id*/, int /*frame_id*/>> asso_epid_frameid;
            IO::ReadPublicPointAssociation(associate_path_, asso_epid_frameid);

            for (auto pt : pts)
            {
                int id = pt.first;
                simulator::EnvPoint *ptr_ep = new simulator::EnvPoint(id, ptr_robot_trajectory_); // MapPoint(id, ptr_robot_trajectory_);
                ptr_ep->GenerateEnvPoint(pt.second);
                // generate measurements
                ptr_ep->AddObservation(ptr_robot_trajectory_->groundtruth_traject_id_Twc_, asso_epid_frameid, b_add_noise_to_meas);

                for (auto ob = ptr_ep->obs_frame_pos_.begin(), ob_end = ptr_ep->obs_frame_pos_.end(); ob != ob_end; ob++)
                {
                    int frame_id = ob->first;
                    Vec3 pos_in_cam = ob->second;
                    asso_frameid_epid_[frame_id].push_back(std::make_pair(id, pos_in_cam));
                }

                if (asso_frameid_epid_.size() > 0) // has observations
                {
                    // association
                    asso_epid_frameid_pixeld_[id] = ptr_ep->obs_frame_pixel_;
                    asso_epid_frameid_pos_[id] = ptr_ep->obs_frame_pos_;
                    // gt
                    vec_points_.push_back(ptr_ep->pos_world_);
                    vec_epid_pos_w_.push_back(std::make_pair(ptr_ep->num_id_, ptr_ep->pos_world_)); // after checking
                }
            }
        }

        void GetBoundsOfPlane(std::vector<Eigen::Vector3d> &vertices, Eigen::Vector3d &max_bounds, Eigen::Vector3d &min_bounds)
        {
            std::vector<double> vec_vertex_x;
            std::vector<double> vec_vertex_y;
            std::vector<double> vec_vertex_z;

            for (auto vertex : vertices)
            {
                vec_vertex_x.push_back(vertex.x());
                vec_vertex_y.push_back(vertex.y());
                vec_vertex_z.push_back(vertex.z());
            }

            auto get_bound = [&](std::vector<double> &vec_vertex_x,
                                 std::vector<double> &vec_vertex_y,
                                 std::vector<double> &vec_vertex_z, bool is_max = true)
            {
                Eigen::Vector3d vec_bound;
                if (is_max)
                {
                    double max_x = *std::max_element(vec_vertex_x.begin(), vec_vertex_x.end());
                    double max_y = *std::max_element(vec_vertex_y.begin(), vec_vertex_y.end());
                    double max_z = *std::max_element(vec_vertex_z.begin(), vec_vertex_z.end());
                    vec_bound << max_x, max_y, max_z;
                }
                else
                {
                    double min_x = *std::min_element(vec_vertex_x.begin(), vec_vertex_x.end());
                    double min_y = *std::min_element(vec_vertex_y.begin(), vec_vertex_y.end());
                    double min_z = *std::min_element(vec_vertex_z.begin(), vec_vertex_z.end());
                    vec_bound << min_x, min_y, min_z;
                }
                return vec_bound;
            };

            max_bounds = get_bound(vec_vertex_x, vec_vertex_y, vec_vertex_z, true);
            min_bounds = get_bound(vec_vertex_x, vec_vertex_y, vec_vertex_z, false);
        }

        void GetMapLandmarks(std::vector<std::pair<int, Vec3>> &vec_epid_pos_w,
                             std::vector<std::pair<int, Eigen::Matrix<double, 3, 2>>> &vec_elid_pos_w)
        {
            vec_epid_pos_w.clear();
            // std::cout<<"size: "<<vec_epid_pos_w_.size()<<std::endl;
            // vec_epid_pos_w = vec_epid_pos_w_;
            // for(int i =0; i<vec_epid_pos_w_.size(); i++)
            //         std::cout<<"sds:"<<vec_epid_pos_w_[i].second<<std::endl;
            for (int i = 0; i < vec_epid_pos_w_.size(); i++)
            {
                vec_epid_pos_w.push_back(std::make_pair(vec_epid_pos_w_[i].first, vec_epid_pos_w_[i].second));
            }
            vec_elid_pos_w.clear();
            // vec_elid_pos_w = vec_elid_pos_w_;

            for (int i = 0; i < vec_elid_pos_w_.size(); i++)
            {
                vec_elid_pos_w.push_back(std::make_pair(vec_elid_pos_w_[i].first, vec_elid_pos_w_[i].second));
            }
        }

        // for visualization
        void GetEnvPoints(std::vector<Vec3> &env_points)
        {
            // without noise
            env_points = vec_points_;
        }

        void GetEnvLines(std::vector<Eigen::Matrix<double, 3, 2>> &env_lines)
        {
            env_lines = vec_lines_;
        }
        void GetEnvLinesStructLabel(std::vector<int> &vec_lines_structlabel)
        {
            vec_lines_structlabel = vec_lines_structlabel_;
        }

        void GetAssoParalis(std::map<int /*mapparaline_id*/, std::vector<int>> &asso_paralineid_elids)
        {
            for (auto mit = asso_paralineid_elids_.begin(); mit != asso_paralineid_elids_.end(); mit++)
            {
                int parali_id = mit->first;
                std::vector<int> maplines;
                for (auto mapline_id : mit->second)
                    maplines.push_back(mapline_id);
                asso_paralineid_elids[parali_id] = maplines;
            }
        }

        void GetAssoMPMeas(std::map<int /*mappoint_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>>> &asso_mp_meas)
        {
            // asso_epid_frameid_pos_
            for (auto mit = asso_epid_frameid_pixeld_.begin(); mit != asso_epid_frameid_pixeld_.end(); mit++)
            {
                int mappoint_id = mit->first;
                std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>> vec_frameid_meas;
                for (auto mit_id = mit->second.begin(); mit_id != mit->second.end(); mit_id++)
                    vec_frameid_meas.push_back(std::make_pair(mit_id->first, Eigen::Vector2d(mit_id->second.x(), mit_id->second.y())));
                asso_mp_meas[mappoint_id] = vec_frameid_meas;
            }
            // TODO: check
            for (auto meas : asso_mp_meas)
            {
                int mp_id = meas.first;

                if (asso_mp_meas[mp_id].size() != asso_epid_frameid_pixeld_[mp_id].size())
                    assert(1 == 0);
                for (auto observation : meas.second)
                {
                    int frame_id = observation.first;
                    // std::cout << "mp_id:" << mp_id << "," << frame_id
                    //           << "," << observation.second
                    //           << "," << asso_epid_frameid_pixeld_[mp_id][frame_id] << std::endl;
                    if (observation.second.x() != asso_epid_frameid_pixeld_[mp_id][frame_id].x())
                    {
                        assert(1 == 0);
                    }
                }
            }
            //
        }

        void GetAssoMLMeas(std::map<int /*mapline_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Vector4d /*2d_meas*/>>> &asso_ml_meas)
        {
            for (auto mit = asso_elid_frameid_pixeld_.begin(); mit != asso_elid_frameid_pixeld_.end(); mit++)
            {
                int mapline_id = mit->first;
                std::vector<std::pair<int /*frame_id*/, Eigen::Vector4d /*2d_meas*/>> vec_frameid_meas;
                for (auto mit_id = mit->second.begin(); mit_id != mit->second.end(); mit_id++)
                {
                    Eigen::Vector4d endpixels;
                    endpixels << mit_id->second(0, 0),
                        mit_id->second(1, 0),
                        mit_id->second(0, 1),
                        mit_id->second(1, 1);
                    // std::cout << "ML meas: " << endpixels.transpose() << std::endl;
                    vec_frameid_meas.push_back(std::make_pair(mit_id->first, endpixels));
                }
                asso_ml_meas[mapline_id] = vec_frameid_meas;
            }
        }

        void GetAssoMLMeasD(std::map<int /*mapline_id*/, std::vector<std::pair<int /*frame_id*/, Eigen::Matrix<double, 6, 1> /*3d_meas_d*/>>> &asso_ml_meas_d)
        {
            // asso_plid_frame_pos_
            // TODO: (empty structure)
            for (auto mit = asso_elid_frameid_pixeld_.begin(); mit != asso_elid_frameid_pixeld_.end(); mit++)
            {
                int mapline_id = mit->first;
                std::vector<std::pair<int /*frame_id*/, Eigen::Matrix<double, 6, 1> /*2d_meas_d*/>> vec_frameid_meas;
                for (auto mit_id = mit->second.begin(); mit_id != mit->second.end(); mit_id++)
                {
                    Eigen::Matrix<double, 6, 1> endpixel_d_s;
                    endpixel_d_s << mit_id->second(0, 0),
                        mit_id->second(1, 0),
                        mit_id->second(2, 0),
                        mit_id->second(0, 1),
                        mit_id->second(1, 1),
                        mit_id->second(2, 1);

                    // std::cout << "ML meas: " << endpixels.transpose() << std::endl;

                    vec_frameid_meas.push_back(std::make_pair(mit_id->first, endpixel_d_s));
                }
                asso_ml_meas_d[mapline_id] = vec_frameid_meas;
            }
        }

        void GetAssoFrameMPs(std::map<int /*kf_id*/, std::vector<int /*mappoint_id*/>> &mp_obsers)
        {
            for (auto frameid_epid : asso_frameid_epid_)
            {
                int frame_id = frameid_epid.first;
                std::vector<int> mappoint_ids;
                for (auto frame_meas : frameid_epid.second)
                {
                    int mappoint_id = frame_meas.first;
                    mappoint_ids.push_back(mappoint_id);
                }
                mp_obsers[frame_id] = mappoint_ids;
            }
            // for (int i = 0; i < asso_frameid_epid_.size(); i++)
            // {
            //     int frame_id = i;
            //     std::vector<int> mappoint_ids;
            //     for (auto mit = asso_frameid_epid_[i].begin(); mit != asso_frameid_epid_[i].end(); mit++)
            //     {
            //         int mappoint_id = mit->first;
            //         mappoint_ids.push_back(mappoint_id);
            //     }
            //     mp_obsers[frame_id] = mappoint_ids;
            // }
        }

        void GetAssoFrameMLs(std::map<int /*kf_id*/, std::vector<int /*mapline_id*/>> &ml_obsers)
        {
            for (int i = 0; i < asso_frameid_elid_.size(); i++)
            {
                int frame_id = i;
                std::vector<int> mapline_ids;
                for (auto mit = asso_frameid_elid_[i].begin(); mit != asso_frameid_elid_[i].end(); mit++)
                {
                    int mapline_id = mit->first;
                    mapline_ids.push_back(mapline_id);
                }
                ml_obsers[frame_id] = mapline_ids;
            }
        }
    };

}
#endif //__VENOM_SRC_MANAGER_EnvManager_HPP__