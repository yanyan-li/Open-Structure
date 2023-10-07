/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-25 05:34:55
 * @LastEditTime: 2022-09-25 05:47:30
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Pose estimation based on measurements.
 * @FilePath: /venom/src/estimator/Track.hpp
 */

#ifndef __VENOM_SRC_TRACK_HPP__
#define __VENOM_SRC_TRACK_HPP__

#include <Eigen/StdVector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "src/manager_env/EnvTrajectory.hpp"
#include "src/utils/UtilStruct.hpp"
#include "src/map/MapVenom.hpp"
#include "src/map/MapLine.hpp"
#include "src/map/MapPoint.hpp"

namespace simulator
{
    class Trajectory;
    class MapLine;
    class MapVeom;
    
    class Track
    {

    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::map<int/*frame_id*/, std::vector<std::pair<int /*mappoint_id*/, Vec3/*posi_in_cam*/>>> obs_point_; // noisy 3D measurement
        std::map<int/*frame_id*/, std::vector<std::pair<int /*mapline_id */, Mat32/*posi_in_cam*/>>> obs_line_;
        
        Trajectory *robot_traject_;
        // landmarks in the map
        std::vector<MapLine *> vec_ptr_mls_;
        std::vector<MapPoint *> vec_ptr_mps_; 
        std::vector<std::pair<int/*mp_id*/, Vec3/*pos_in_w*/>> vec_epid_pos_w_; 
        std::vector<std::pair<int, Mat32>> vec_elid_pos_w_;
        std::map<int/*mappoint_id*/, Vec3/*posi_in_w*/> local_mappoints_;
        std::map<int/*maplines_id*/, Mat32/*posi_in_w*/> local_maplines_;
        std::map<int /*envparaline_id*/, std::vector<int /*envline_id*/>> paralineid_elids_;

        // poses
        std::vector<std::pair<int /*frame_id*/, Eigen::Matrix4d /*frame_pose*/>> vec_localmap_Twc_;
        std::vector<std::pair<int/*frame_id*/,  Eigen::Matrix4d/*frame_pose*/>> vec_vo_Twc_;

    public:
        std::vector<double> tri_point_inverse_depth_;
        std::vector<Vec3> tri_point_xyz_; // reconstructured 3D mappoints
        
        std::vector<std::pair<Vec3/*mappoint_id*/,std::vector<int/*frame_id*/> >> tri_point_observations_;
        
        std::vector<std::pair<int/*mappoint_id*/, Vec3>> vec_mpid_xyz_;
        std::vector<std::pair<int/*mappoint_id*/, std::vector<int/*frame_id*/>>> vec_mpid_frameid_;
        std::mutex mMutexSpeed;
        int num_track_speed_;
        bool mbTrackOdometry;

    public:
        /**
         * @brief
         *
         */
        Track(std::map<int /*envpoint_id*/, frame_point_meas> &asso_epid_frameid_pos,
              std::map<int /*frame_id*/, std::vector<std::pair<int /*ep_id*/, Vec3>>> &asso_frameid_epid, // noisy 3D in camera coordinate
              std::map<int /*envline_id*/, frame_line_meas> &asso_elid_frameid_pos,
              std::map<int /*frame_id*/, std::vector<std::pair<int /*el_id*/, Mat32>>> &asso_frameid_elid, // noisy 3D in camera coordinate
              std::map<int /*envparaline_id*/, std::vector<int /*envline_id*/>> asso_paralineid_elids_,
              Trajectory *robot_trajectory) : robot_traject_(robot_trajectory), paralineid_elids_(asso_paralineid_elids_)
        {
            // initialization measurements
            obs_point_.clear();
            obs_line_.clear();
            for (auto frameid_epid : asso_frameid_epid)
            {
                int frame_idx = frameid_epid.first;
                for (auto meas : frameid_epid.second)
                {
                    int ep_id = meas.first;
                    Vec3 pos_in_cam = meas.second;
                    double distance = pos_in_cam.norm();
                    if (distance < 0.01)
                        assert(1 == 0);
                    obs_point_[frame_idx].push_back(std::make_pair(ep_id, pos_in_cam));
                }
            }
            for (auto frameid_elid : asso_frameid_elid)
            {
                int frame_idx = frameid_elid.first;
                for (auto meas : frameid_elid.second)
                {
                    int el_id = meas.first;
                    Mat32 pos_in_cam = meas.second;
                    double distance = (pos_in_cam.col(0) - pos_in_cam.col(1)).norm();
                    if (distance < 0.01)
                    {
                        std::cout << "short distance:" << distance << ",frame:" << frame_idx << ",el_id" << el_id << std::endl;
                        assert(0 == 1);
                    }

                    obs_line_[frame_idx].push_back(std::make_pair(el_id, pos_in_cam));
                }
            }

#ifdef __DEBUG__OFF
            for(auto frame_ep:asso_frameid_epid)
            {
                int frame_idx = frame_ep.first;
                for(int i=0; i<frame_ep.second.size(); i++)
                    assert(frame_ep.second[i].first == obs_point_[frame_idx][i].first);
                    // std::cout<<"ssd:"<<obs_point_[frame_idx].size()
                    //          <<", "<<frame_ep.second[i].first<< ","<< frame_ep.second[i].second.x()
                    //          <<", "<< obs_point_[frame_idx][i].first
                    //          <<","<<obs_point_[frame_idx][i].second.x()<<std::endl;
            }
#endif      
            // initialize lcoal_mappoints      
            for(auto mit=asso_epid_frameid_pos.begin(); mit!=asso_epid_frameid_pos.end(); mit++)
            {
                int mappoint_id = mit->first;
                local_mappoints_[mappoint_id] = Vec3::Zero();
            }
            // initialize local_maplines
            for(auto mit = asso_elid_frameid_pos.begin(); mit!=asso_elid_frameid_pos.end(); mit++)
            {
                int mapline_id = mit->first;
                local_maplines_[mapline_id] = Mat32::Zero();
            }

            mbTrackOdometry = false;
            num_track_speed_ = 1;
        }

        void SetMapLandmarks(std::vector<std::pair<int, Vec3>> &vec_epid_pos_w)
        {
            if (false)
                vec_epid_pos_w_ = vec_epid_pos_w; 
            else
            {
               //    vec_epid_pos_w_ = std::vector<std::pair<int, Vec3>>(vec_epid_pos_w.size(), std::pair<int, Vec3> );
               double max_nt = 0.051;
               std::random_device rd;
               std::default_random_engine generator(rd());
               std::normal_distribution<double> euc_n(0, max_nt);

               for(size_t i =0; i<vec_epid_pos_w.size(); i++)
               {
                   Vec3 pt;                   
                   pt<<vec_epid_pos_w[i].second(0, 0) + euc_n(generator),
                       vec_epid_pos_w[i].second(1, 0) + euc_n(generator),
                       vec_epid_pos_w[i].second(2, 0) + euc_n(generator);
                   vec_epid_pos_w_.push_back(std::make_pair(vec_epid_pos_w[i].first, pt)); 
#ifdef __DEBUG__OFF
                    std::cout<<"SetMapLandmarks:"<<vec_epid_pos_w[i].second(0, 0)<< ","
                    <<vec_epid_pos_w[i].second(1, 0)<<","
                    <<vec_epid_pos_w[i].second(2, 0)<<";"<<"\n"
                    <<"with noise:"<<pt(0, 0)<< ","<<pt(1, 0)<<","<<pt(2, 0)<<std::endl; 
#endif                    
               }
            }
        }

        void SetMapLandmarks(std::vector<std::pair<int, Mat32>>& vec_elid_pos_w)
        {
            vec_elid_pos_w_ = vec_elid_pos_w;
        }

        /**
         * @brief pose estimation based on traditional methods
         *
         */
        void TrackOdometryPtr(std::string root_path, bool with_localmap, bool with_lines, std::vector<Eigen::Matrix4d> &initial_poses)
        {
            std::vector<std::pair<int /*frame_id*/, Eigen::Matrix4d /*frame_pose*/>> vec_vo_estimated_Twc;
            std::vector<std::pair<int /*frame_id*/, Eigen::Matrix4d /*frame_pose*/>> vec_gt_Twc;

            std::vector<Vec3> vec_mappoint;
            for (auto frame : robot_traject_->groundtruth_traject_id_Twc_)
            {
                int frame_id = frame.first;

                Mat4 vo_prev_pose_in_world, vo_curr_pose_in_world;
                Mat4 map_prev_pose_in_world, map_curr_pose_in_world;
                Mat4 curr_gt_pose_in_world;

                if (frame_id == 0) // initialization
                {
                    curr_gt_pose_in_world = frame.second;
                    vo_curr_pose_in_world = frame.second;
                    map_curr_pose_in_world = frame.second;

                    for (auto frame_info : obs_point_)
                    {
                        int frame_idx = frame_info.first;
                        if (frame_idx != frame_id)
                            continue;
                        for (auto frame_meas : frame_info.second)
                        {
                            int point_id = frame_meas.first;
                            Vec3 point_in_cam = frame_meas.second;
                            // from camera to world
                            Vec3 point_in_w = vo_curr_pose_in_world.block(0, 0, 3, 3) * point_in_cam +
                                              vo_curr_pose_in_world.block(0, 3, 3, 1); // 这里有问题，应该是世界坐标系

                            local_mappoints_[point_id] = point_in_w;
#ifdef __DEBUG__OFF
                            for (auto epid_pos : vec_epid_pos_w_)
                                if (epid_pos.first == point_id)
                                    std::cout << "point distance:" << (epid_pos.second - point_in_w).norm() << std::endl;
#endif
                        }
                    }

                    for (auto frame_info : obs_line_)
                    {
                        int frame_idx = frame_info.first;
                        if (frame_idx != frame_id)
                            continue;
                        for (auto frame_meas : frame_info.second)
                        {
                            int line_id = frame_meas.first;
                            Mat32 line_in_cam = frame_meas.second;
                            // from camera to world
                            Mat32 t_in_world;
                            t_in_world.block(0, 0, 3, 1) = vo_curr_pose_in_world.block(0, 3, 3, 1);
                            t_in_world.block(0, 1, 3, 1) = vo_curr_pose_in_world.block(0, 3, 3, 1);

                            Mat32 line_in_w = vo_curr_pose_in_world.block(0, 0, 3, 3) * line_in_cam + t_in_world;
                            local_maplines_[line_id] = line_in_w;
                        }
                    }
                }
                else
                {
                    bool b_skip_fusion = false;
                    DrawMeasurements(obs_point_[frame_id], obs_line_[frame_id], frame_id);
                    // relative pose  T_{prev, curr}
                    vo_curr_pose_in_world = RelativePoseEstimation(vo_prev_pose_in_world,
                                                                   obs_point_[frame_id - 1],
                                                                   obs_line_[frame_id - 1],
                                                                   obs_point_[frame_id],
                                                                   obs_line_[frame_id]);
                    if (with_localmap)
                    {
                        map_curr_pose_in_world = LocalMapPoseEstimation(local_mappoints_,
                                                                        obs_point_[frame_id],
                                                                        frame.second,
                                                                        frame_id); //
                        // robot_traject_->vec_traject_gt_Twc_[frame_id]);
                        if (map_curr_pose_in_world.isIdentity())
                        {
                            std::cout << "problems in localmap_pose" << std::endl;
                            std::cout << "\033[0;31mThis is the " << frame_id << "th frame.\033[0m" << std::endl;
                            std::cout << "obs_point_[frame_id]:" << obs_point_[frame_id].size() << std::endl;

                            for (int i = 0; i < obs_point_[frame_id].size(); i++)
                                std::cout << obs_point_[frame_id].size() << ","
                                          << obs_point_[frame_id][i].first << ","
                                          << obs_point_[frame_id][i].second(0) << ","
                                          << obs_point_[frame_id][i].second(1) << ","
                                          << obs_point_[frame_id][i].second(2) << std::endl;

                            // use last_frame's pose
                            map_curr_pose_in_world = map_prev_pose_in_world;
                            b_skip_fusion = true;
                        }
                    }

                    if (!with_lines && !b_skip_fusion)
                    {
                        if (with_localmap)
                            FuseMapPoint(local_mappoints_, map_curr_pose_in_world, obs_point_[frame_id]);
                        else
                            FuseMapPoint(local_mappoints_, vo_curr_pose_in_world, obs_point_[frame_id]);
                    }
                    else if (with_lines && !b_skip_fusion)
                    {
                        if (with_localmap)
                        {
                            FuseMapPoint(local_mappoints_, map_curr_pose_in_world, obs_point_[frame_id]);
                            FuseMapLine(local_maplines_, map_curr_pose_in_world, obs_line_[frame_id]);
                        }
                        else
                        {
                            FuseMapPoint(local_mappoints_, vo_curr_pose_in_world, obs_point_[frame_id]);
                            FuseMapLine(local_maplines_, vo_curr_pose_in_world, obs_line_[frame_id]);
                        }
                    }
                    curr_gt_pose_in_world = frame.second;
                }
                // save previous pose
                map_prev_pose_in_world = map_curr_pose_in_world;
                vo_prev_pose_in_world = vo_curr_pose_in_world;

                vec_vo_Twc_.push_back(std::make_pair(frame_id, vo_curr_pose_in_world));
                vec_gt_Twc.push_back(std::make_pair(frame_id, curr_gt_pose_in_world));
                vec_localmap_Twc_.push_back(std::make_pair(frame_id, map_curr_pose_in_world));
#ifdef __VERBOSE__NOT
                std::cout << "ground truth pose:" << std::endl
                          << curr_gt_pose_in_world << std::endl;
                std::cout << "predicted pose (T_{w,i}):" << std::endl
                          << vo_prev_pose_in_world << std::endl;
                // std::cout << "relative ground truth pose:" << std::endl
                //           << pose_gt_curr_in_prev << std::endl;
                // std::cout << "relative pose (T_{i-1,i}):" << std::endl
                //           << vo_curr_pose_in_world << std::endl;
                std::cout << "predicted pose in map (T_{w,i}):" << std::endl
                          << map_curr_pose_in_world << std::endl;
#endif
                // insert pose into map_viewer
                InsertFramePosesVect(initial_poses, map_curr_pose_in_world);
            }
            mbTrackOdometry = true;
            cv::destroyAllWindows();

            SaveFramePredictedTrajectoryTUM(root_path + "estimated_vo.txt", vec_vo_Twc_);
            SaveFramePredictedTrajectoryTUM(root_path + "ground_pose.txt", vec_gt_Twc);
            SaveFramePredictedTrajectoryTUM(root_path + "estimated_localmap.txt", vec_localmap_Twc_);
        }

        Eigen::Matrix4d LocalMapPoseEstimation(std::vector<std::pair<int /*point_id*/, Vec3>> &vec_epid_pos_w,
                                               std::vector<std::pair<int /*point_id*/, Vec3>> &points_curr,
                                               Eigen::Matrix4d &gt_pose,
                                               const int frame_id)
        {
            // PnP
            std::map<int, std::pair<Vec3, Eigen::Vector2d>> vec_data_pnp;
            vec_data_pnp.clear();
            // GetMapLandmarks
            for (size_t i = 0; i < vec_epid_pos_w.size(); i++)
            {
                // id
                int prev_point_id = vec_epid_pos_w[i].first;
                // 3D env point
                Vec3 point_in_map = vec_epid_pos_w[i].second;

                int count = 0;
                for (size_t j = 0; j < points_curr.size(); j++)
                {
                    // id
                    int curr_point_id = points_curr[j].first;
                    // if (curr_point_id <1)
                    //     continue;
                    if (prev_point_id == curr_point_id)
                    {
                        // std::cout<< i<<","<< points_curr.size()<<","<<vec_data_pnp.size()<<","<<count<<",prev_point_id:"<<prev_point_id<<","<<curr_point_id<<std::endl;
                        // 2D pixel of the curr_frame
                        Vec3 point_in_curr = points_curr[j].second;
                        assert(point_in_curr.z() > 0.01);
                        Eigen::Vector2d pixel_curr;
                        GetPixelPosition(point_in_curr, pixel_curr);
                        vec_data_pnp[curr_point_id] = std::make_pair(point_in_map, pixel_curr);
                        count++;
                    }
                }
            }

            // pnp for pose estimation
            if (vec_data_pnp.size() < 5)
            {
                std::cout << "not enough for pnp" << std::endl;
                return Eigen::Matrix4d::Identity();
            }
            Eigen::Matrix4d pose = PnPComputation(vec_data_pnp);

            // if((gt_pose.block(0,3,3,1) -  pose.block(0,3,3,1)).norm()>1 )
            // {
            //     std::cout<<"traj size:"<<vec_data_pnp.size()<<std::endl;
            DrawFeatures(vec_data_pnp, gt_pose, frame_id);
            //     std::cout<<"translation: "<<gt_pose.block(0,3,3,1)<<","<<pose.block(0,3,3,1)<<std::endl;
            // }
            return pose;
        }

        Eigen::Matrix4d LocalMapPoseEstimation(std::map<int, Vec3> &epid_pos_localmap,
                                               std::vector<std::pair<int /*point_id*/, Vec3>> &points_curr,
                                               Eigen::Matrix4d &gt_pose,
                                               const int frame_id)
        {
            // std::cout << ">>>> tracking based on the map-to-frame strategy. " << std::endl;
            // PnP
            std::map<int, std::pair<Vec3, Eigen::Vector2d>> vec_data_pnp;
            vec_data_pnp.clear();
            // GetMapLandmarks
            for (auto epit = epid_pos_localmap.begin(); epit != epid_pos_localmap.end(); epit++)
            {
                int prev_point_id = epit->first;
                Vec3 point_in_map = epit->second;

                // remove uninitialized points
                if (point_in_map.isZero())
                    continue;
                double point_distance = point_in_map.norm();
                if (point_distance < 0.01)
                    assert(0 == 1);
                for (size_t j = 0; j < points_curr.size(); j++)
                {
                    // id
                    int curr_point_id = points_curr[j].first;
                    if (prev_point_id == curr_point_id) // map_point --- frame_point matching
                    {
                        // 2D pixel of the curr_frame
                        Vec3 point_in_curr = points_curr[j].second;
                        if (point_in_curr.z() <= 0)
                            assert(1 == 0);
                        Eigen::Vector2d pixel_curr;
                        GetPixelPosition(point_in_curr, pixel_curr);

                        vec_data_pnp[prev_point_id] = std::make_pair(point_in_map, pixel_curr);
                    }
                }
            }

            if (vec_data_pnp.size() < 6)
            {
                std::cout << "not enough for pnp" << std::endl;
                return Eigen::Matrix4d::Identity();
            }

            Eigen::Matrix4d pose = PnPComputation(vec_data_pnp);

            return pose;
        }

        /**
         * @brief
         *
         * @param pose_prev_in_w
         * @param points_prev
         * @param lines_prev
         * @param points_curr
         * @param lines_curr
         * @return Eigen::Matrix4d
         */
        Eigen::Matrix4d RelativePoseEstimation(Eigen::Matrix4d &pose_prev_in_w,
                                               std::vector<std::pair<int /*point_id*/, Vec3>> &points_prev,
                                               std::vector<std::pair<int /*line_id*/, Mat32>> &lines_prev,
                                               std::vector<std::pair<int /*point_id*/, Vec3>> &points_curr,
                                               std::vector<std::pair<int /*line_id*/, Mat32>> &lines_curr)
        {

            Eigen::Matrix4d pose_curr_in_w = Eigen::Matrix4d::Identity();
            // PnP
            std::map<int, std::pair<Vec3, Eigen::Vector2d>> vec_data_pnp;
            for (int i = 0; i < points_prev.size(); i++)
            {
                // id
                int prev_point_id = points_prev[i].first;
                // 3D point
                Vec3 point_in_prev = points_prev[i].second;
                // from camera to world
                Vec3 point_in_w = pose_prev_in_w.block(0, 0, 3, 3) * point_in_prev + pose_prev_in_w.block(0, 3, 3, 1); // 这里有问题，应该是世界坐标系
                for (size_t j = 0; j < points_curr.size(); j++)
                {
                    // id
                    int curr_point_id = points_curr[j].first;
                    if (prev_point_id == curr_point_id)
                    {
                        // 2D pixel of the curr_frame
                        Vec3 point_in_curr = points_curr[j].second;
                        Eigen::Vector2d pixel_curr;
                        GetPixelPosition(point_in_curr, pixel_curr);
                        //
                        vec_data_pnp[prev_point_id] = std::make_pair(point_in_w, pixel_curr);

                        break;
                    }
                }
            }

            // pnp for pose estimation

            if (vec_data_pnp.size() < 6)
            {
                std::cout << "not enough for pnp" << std::endl;
                return pose_prev_in_w;
            }
            else
            {
                pose_curr_in_w = PnPComputation(vec_data_pnp);
            }

            return pose_curr_in_w;
        }

        void FuseMapPoint(std::map<int /*mappoint_id*/, Vec3 /*posi_in_w*/> &local_map,
                          Eigen::Matrix4d &pose_curr_in_w,
                          std::vector<std::pair<int /*point_id*/, Vec3>> &points_curr)
        {
            for (int j = 0; j < points_curr.size(); j++)
            {
                int curr_point_id = points_curr[j].first;
                Vec3 point_in_curr = points_curr[j].second;
                Vec3 new_mappoint = pose_curr_in_w.block(0, 0, 3, 3) * point_in_curr + pose_curr_in_w.block(0, 3, 3, 1); // 这里有问题，应该是世界坐标系

                Vec3 point_in_w = local_map[curr_point_id];
                // valid
                if (point_in_w.isZero())
                {
                    // new mappoint detected
                    local_map[curr_point_id] = new_mappoint;
                }
                else
                {
                    // fuse
                    // TODO: (weights)
                    point_in_w = (0.8 * point_in_w + 0.2 * new_mappoint);
                    local_map[curr_point_id] = point_in_w;
                }
            }
        }

        void FuseMapLine(std::map<int /*mapline_id*/, Mat32> &local_map,
                         Eigen::Matrix4d &pose_curr_in_w,
                         std::vector<std::pair<int /*line_id*/, Mat32 /*posi_in_cam*/>> &lines_curr)
        {
            for (size_t j = 0; j < lines_curr.size(); j++)
            {
                int curr_line_id = lines_curr[j].first;
                Mat32 line_in_curr = lines_curr[j].second;

                Vec3 new_mapline_s = pose_curr_in_w.block(0, 0, 3, 3) * line_in_curr.col(0) + pose_curr_in_w.block(0, 3, 3, 1);
                Vec3 new_mapline_e = pose_curr_in_w.block(0, 0, 3, 3) * line_in_curr.col(1) + pose_curr_in_w.block(0, 3, 3, 1);

                Vec3 new_direction = new_mapline_s - new_mapline_e;
                double new_distance = new_direction.norm();
                // std::cout << "lenght:" << new_direction.norm() << std::endl;
                Eigen::Vector3d new_normal = new_mapline_s.cross(new_mapline_e);
                new_normal = new_normal.normalized();
                new_direction = new_direction.normalized();

                Mat32 new_mapline;
                new_mapline.col(0) = new_mapline_s;
                new_mapline.col(1) = new_mapline_e;

                Mat32 line_in_w = local_map[curr_line_id];

                if (line_in_w.isZero())
                {
                    local_map[curr_line_id] = new_mapline;
                }
                else 
                {   //TODO:(weights)

                    // fuse
                    // line_in_w = (line_in_w+new_mapline)/2;
                    // local_map[curr_line_id] = line_in_w;
                    double localmap_distance = (line_in_w.col(0) - line_in_w.col(1)).norm();
                    Mat32 new_endpoints;
                    new_endpoints.col(0) = new_mapline_s;
                    new_endpoints.col(1) = new_mapline_e;

                    if (localmap_distance > new_distance)
                    {
                        new_endpoints.col(0) = line_in_w.col(0);
                        new_endpoints.col(1) = line_in_w.col(1);
                    }

                    // Eigen ::Vector3d EndPoints3d_i_s_distance = new_mapline.col(0) - line_in_w.col(0);

                    Vec3 mid_point = (new_endpoints.col(0) + new_endpoints.col(1)) / 2;
                    // from 1 to 0
                    Vec3 direction_w = (line_in_w.col(0) - line_in_w.col(1)).normalized();
                    if (direction_w.dot(new_direction) > 0.85)
                        direction_w = (0.8 * direction_w + 0.2 * new_direction).normalized();
                    else if (direction_w.dot(new_direction) < -0.85)
                        direction_w = (0.8 * direction_w - 0.2 * new_direction).normalized();

                    Vec3 direc_mid_s = mid_point - new_endpoints.col(1);
                    local_map[curr_line_id].col(1) = mid_point - (direc_mid_s.dot(direction_w)) * direction_w;

                    Vec3 direc_mid_e = new_endpoints.col(0) - mid_point;
                    local_map[curr_line_id].col(0) = mid_point + (direc_mid_e.dot(direction_w)) * direction_w;

                    Eigen::Vector3d EndPoints3d_i_e_distance = new_mapline.col(1) - new_endpoints.col(0);
                }
            }
        }

        void GetPixelPosition(Vec3 &point_in_curr, Eigen::Vector2d &pixel_curr)
        {

            float u = robot_traject_->cam_intri.fx * (point_in_curr / point_in_curr(2, 0))(0, 0) + robot_traject_->cam_intri.cx;
            float v = robot_traject_->cam_intri.fy * (point_in_curr / point_in_curr(2, 0))(1, 0) + robot_traject_->cam_intri.cy;
            pixel_curr << u, v;
        }

        Eigen::Matrix4d PnPComputation(std::map<int, std::pair<Vec3, Eigen::Vector2d>> &vec_data_pnp)
        {
            cv::Mat R_curr_in_prev, r_curr_in_prev, t_curr_in_prev;
            std::vector<cv::Point2d> vec_pixel_curr;
            // 3D model points.
            std::vector<cv::Point3d> vec_point_in_world;
            for (auto data_pnp:vec_data_pnp)
            {
                int mp_id = data_pnp.first;

                Eigen::Vector2d pixel = data_pnp.second.second;
                vec_pixel_curr.push_back(cv::Point2d(pixel(0, 0), pixel(1, 0)));
                Vec3 point = data_pnp.second.first;
                vec_point_in_world.push_back(cv::Point3d(point(0, 0), point(1, 0), point(2, 0)));
            }

            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << robot_traject_->cam_intri.fx, 0, robot_traject_->cam_intri.cx,
                                     0, robot_traject_->cam_intri.fy, robot_traject_->cam_intri.cy,
                                     0, 0, 1);
            cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

            bool result = cv::solvePnP(vec_point_in_world, vec_pixel_curr, camera_matrix, dist_coeffs, r_curr_in_prev, t_curr_in_prev,false, cv::SOLVEPNP_EPNP);
            // bool result = cv::solvePnPRansac(vec_point_in_world, vec_pixel_curr, camera_matrix, dist_coeffs, r_curr_in_prev, t_curr_in_prev,false, 100, 2);
            Eigen::Matrix4d T_curr_in_prev =Eigen::Matrix4d::Identity();
            
            if(!result)
                return T_curr_in_prev;

            cv::Rodrigues(r_curr_in_prev, R_curr_in_prev);

            R_curr_in_prev = R_curr_in_prev.t();
            t_curr_in_prev = -R_curr_in_prev * t_curr_in_prev; // translation of inverse
            Eigen::Matrix3d R_in_prev;
            cv::cv2eigen(R_curr_in_prev, R_in_prev);
            T_curr_in_prev.block(0,0,3,3) = R_in_prev;
            T_curr_in_prev(0,3) = t_curr_in_prev.at<double>(0,0);
            T_curr_in_prev(1,3) = t_curr_in_prev.at<double>(1,0);
            T_curr_in_prev(2,3) = t_curr_in_prev.at<double>(2,0);

            return T_curr_in_prev;
        }

       
        /**
         * @brief 
         * 
         * @param point_obs 
         * @param Twcs 
         */
        void Triangulation(std::vector/*mappoint_id*/<std::vector/*<frame_id, mp_c>*/<std::pair<int, Vec3>>> point_obs, std::vector<Eigen::Matrix4d> Twcs)
        {
            tri_point_xyz_ = std::vector<Vec3>(point_obs.size(), Vec3::Zero());
            
            int reconstructed_xyz_id = point_obs.size();
            int mappoint_id = -1;
            // observations
            for (auto &ob : point_obs)
            {
                mappoint_id += 1;

                Vec3 point_camera;
                // less measurements
                if (ob.size() < 3)
                    continue; 

                reconstructed_xyz_id += 1;    

                Eigen::MatrixXd A(ob.size() * 2, 4);
                int index = 0;
                Eigen::Matrix4d Twc0 = Twcs[ob[0].first];
                //Vec3 ob_0 = ob[0].second;
                
                // observation of the mappoint
                std::vector<int> observations;
                observations.push_back(ob[0].first);
                for (size_t i = 1; i < ob.size(); ++i)
                {
                    Vec3 ob0 = ob[i].second;
                    ob0 /= ob0(2, 0);
                    // P = T_cs_c0 : from c0 to cs
                    Eigen::Matrix4d P = Twcs[ob[i].first].inverse() * Twc0;

                    // Vec3 ob_00 = P.inverse().block(0,0,3,3)*ob0+ P.inverse().block(0,3,3,1);
                    // std::cout<<"camera 2: "<<ob_00<<std::endl;
                    // Vec3 f = ob0/ ob0(2,0);//.normalized(); //.normalized();
                    // std::cout<<" f "<< f<< ", "<< ob0<<std::endl;
                    A.row(index++) = ob0(0, 0) * P.row(2) - ob0(2, 0) * P.row(0);
                    A.row(index++) = ob0(1, 0) * P.row(2) - ob0(2, 0) * P.row(1);
                    
                    observations.push_back(ob[i].first);
                }
                Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();
                point_camera = svd_V.head(3) / svd_V(3);
                Eigen::Matrix3d Rwc = Twc0.block(0, 0, 3, 3);
                Vec3 twc = Twc0.block(0, 3, 3, 1);
                Vec3 point_w = Rwc * point_camera + twc;
                tri_point_inverse_depth_.push_back(1 / point_camera(2));
                tri_point_xyz_.push_back(point_w);

                tri_point_observations_.push_back(std::make_pair(point_w, observations));

                vec_mpid_xyz_.push_back(std::make_pair(reconstructed_xyz_id, point_w));
                vec_mpid_frameid_.push_back(std::make_pair(reconstructed_xyz_id, observations));
            }
        }

       /**
        * @brief Get the Init Map Point object
        * 
        * @param mappoints 
        */
        void GetInitMapPoints(std::map<int /*mappoint_id*/, Vec3/*posi_w*/> &mappoints)
        {
            for(auto mit=local_mappoints_.begin(); mit!=local_mappoints_.end(); mit++)
            {
                int mappoint_id = mit->first;
                if (mit->second == Vec3::Zero()) // remove empty landmarks
                    continue;
                double point_distance = mit->second.norm();
                if (point_distance < 0.1)
                    assert(1 == 0);
                // continue;
                mappoints[mappoint_id] = mit->second;
            }
        }

        void GetInitFramePoses(std::map<int /*kf_id*/, Mat4> &kfs, bool use_vo_pose = true)
        {
            if (!use_vo_pose)
            {
                for (auto mit = vec_localmap_Twc_.begin(); mit != vec_localmap_Twc_.end(); mit++)
                {
                    int frame_id = mit->first;
                    Mat4 frame_pose = mit->second;
                    kfs[frame_id] = frame_pose;
                }
            }
            else
            {
                for (auto mit = vec_vo_Twc_.begin(); mit != vec_vo_Twc_.end(); mit++)
                {
                    int frame_id = mit->first;
                    Mat4 frame_pose = mit->second;
                    kfs[frame_id] = frame_pose;
                }
            }
        }

        void GetInitFramePosesVect(std::vector<Mat4> &kfs)
        {
            // for(auto mit=vec_localmap_Twc_.begin(); mit!=vec_localmap_Twc_.end(); mit++)
            // {
            //     int frame_id = mit->first;
            //     Mat4 frame_pose = mit->second;
            //     kfs.push_back(frame_pose);
            // }
            kfs.clear();
            for (auto mit = vec_vo_Twc_.begin(); mit != vec_vo_Twc_.end(); mit++)
            {
                int frame_id = mit->first;
                Mat4 frame_pose = mit->second;
                std::cout << "*****************" << frame_pose << std::endl;
                kfs.push_back(frame_pose);
            }
        }

        void InsertFramePosesVect(std::vector<Mat4> &kfs, Mat4 kf)
        {
            std::unique_lock<std::mutex> lock(mMutexSpeed);
            kfs.push_back(kf);
        }

        /**
         * @brief Get the Init Map Lines object
         * 
         * @param maplines 
         */
        void GetInitMapLines(std::map<int /*maplines_id*/, Mat32/*posi_w*/> &maplines)
        {
            for(auto mit=local_maplines_.begin(); mit!=local_maplines_.end(); mit++)
            {
                int mapline_id = mit->first;
                if (mit->second == Mat32::Zero()) // remove empty landmarks
                    continue;

                double line_length = (mit->second.col(0) - mit->second.col(1)).norm();
                if (line_length < 0.01)
                    assert(1 == 0);
                maplines[mapline_id] = mit->second;
            }
        }

        void SaveFrameGTTrajectoryTUM(const std::string &filename)
        {
            std::cout << "\033[0;33m[Venom Simulator Printer] Saving keyframe trajectory to " << filename << ".\033[0m" << std::endl;
            
            std::ofstream pose_file;
            pose_file.open(filename);
            // for (size_t i = 0, i_end = robot_traject_->vec_traject_gt_Twc_.size(); i < i_end; i++)
            for(auto frame: robot_traject_->groundtruth_traject_id_Twc_)
            {
                Eigen::Matrix4d Twc_i = frame.second;
                Eigen::Matrix3d R = Twc_i.block(0,0,3,3);

                Eigen::Quaterniond quat(R);
                Vec3 trans = Twc_i.block(0, 3, 3, 1);
                pose_file << frame.first << " " << trans(0) << " " << trans(1) << " " << trans(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
            }

            pose_file.close();
        }

        void SaveFramePredictedTrajectoryTUM(const std::string &filename, std::vector<std::pair<int /*frame_id*/, Eigen::Matrix4d /*frame_pose*/>> &vec_Twcs)
        {
            std::cout << "\033[0;33m[Venom Simulator Printer] Saving keyframe trajectory to " << filename << ".\033[0m" << std::endl;
            
            std::ofstream pose_file;
            pose_file.open(filename);
            //std::sort(vec_Twcs.begin(), vec_Twcs.end());
            sort(vec_Twcs.begin(),vec_Twcs.end(), [](const std::pair<int, Eigen::Matrix4d>& a, const std::pair<int, Eigen::Matrix4d>& b){ return a.first < b.first; }); // lambda way

            for (size_t i = 0, i_end = vec_Twcs.size(); i < i_end; i++)
            {   
                int frame_id = vec_Twcs[i].first;
                Eigen::Matrix4d  Twc_i = vec_Twcs[i].second;
                Eigen::Matrix3d R = Twc_i.block(0,0,3,3);
                Eigen::Quaterniond quat(R);

                Vec3 trans = Twc_i.block(0, 3, 3, 1);
                pose_file << frame_id << " " << trans(0) << " " << trans(1) << " " << trans(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
            }

            pose_file.close();
        }

        void DrawMeasurements(std::vector<std::pair<int /*point_id*/, Vec3>> &points_curr,
                              std::vector<std::pair<int /*line_id*/, Mat32>> &lines_curr,
                              int frame_idx)
        {
            bool b_show_feature_id = true;
            bool b_save_meas_img = true;
            // write image
            cv::Mat img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
            int font_face = cv::FONT_HERSHEY_COMPLEX;
            double font_scale = 0.5;
            int thickness = 1;
            std::string text = "frame " + std::to_string(frame_idx);
            cv::putText(img, text, cv::Point(30, 30), font_face, font_scale, cv::Scalar(125, 0, 125), thickness, 8, 0);

            // 2D features
            for (auto point_i : points_curr)
            {
                int mp_id = point_i.first;
                Vec3 point_in_c = point_i.second;
                // Eigen::Matrix4d Tcw = gt_pose.inverse();
                Eigen::Vector2d pixel_curr;
                // std::cout<<"point_in_c:"<<point_in_c.z()<<std::endl;
                GetPixelPosition(point_in_c, pixel_curr);
                cv::circle(img, cv::Point(pixel_curr(0), pixel_curr(1)), 1,
                           cv::Scalar(0, 255, 0));
                if (b_show_feature_id)
                    cv::putText(img, std::to_string(mp_id), cv::Point(pixel_curr(0), pixel_curr(1)), font_face, font_scale, cv::Scalar(0, 255, 0), thickness, 8, 0);
            }

            for (auto line_i : lines_curr)
            {
                int ml_id = line_i.first;
                Vec3 s_line_in_c = line_i.second.col(0);
                Vec3 e_line_in_c = line_i.second.col(1);
                Eigen::Vector2d s_pixel_curr, e_pixel_curr;
                GetPixelPosition(s_line_in_c, s_pixel_curr);
                GetPixelPosition(e_line_in_c, e_pixel_curr);
                cv::Scalar color = cv::Scalar(0, 0, 255);
                GetParallelLabel(ml_id, color);
                cv::line(img, cv::Point(s_pixel_curr(0), s_pixel_curr(1)), cv::Point(e_pixel_curr(0), e_pixel_curr(1)), color, 2);
                if (b_show_feature_id)
                    cv::putText(img, std::to_string(ml_id), cv::Point(s_pixel_curr(0), s_pixel_curr(1)), font_face, font_scale, color, thickness, 8, 0);
            }
            cv::namedWindow("2D Viewer", cv::WINDOW_NORMAL);
            cv::imshow("2D Viewer", img);
            cv::waitKey(int(50 / num_track_speed_));
            // if (cv::waitKey() == 27)
            if (b_save_meas_img)
                cv::imwrite(std::to_string(frame_idx) + ".png", img);
        }

        void DrawFeatures(std::map<int, std::pair<Vec3, Eigen::Vector2d>> &vec_data_pnp,
                          Eigen::Matrix4d &gt_pose, int frame_idx)
        {
            cv::Mat img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
            // cv::Mat image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
            // image.setTo(cv::Scalar(100, 0, 0));
            int font_face = cv::FONT_HERSHEY_COMPLEX;
            double font_scale = 0.5;
            int thickness = 1;
            std::string text = "frame " + std::to_string(frame_idx);
            cv::putText(img, text, cv::Point(30, 30), font_face, font_scale, cv::Scalar(125, 0, 125), thickness, 8, 0);

            // 2D features
            for (auto data_pnp : vec_data_pnp)
            {
                int mp_id = data_pnp.first;
                Vec3 point_w = data_pnp.second.first;
                Eigen::Matrix4d Tcw = gt_pose.inverse();

                Eigen::Matrix3d R = Tcw.block(0, 0, 3, 3);
                Eigen::Vector3d t = Tcw.block(0, 3, 3, 1);
                Vec3 point_in_c = R * point_w + t; // 这里有问题，应该是世界坐标系
                Eigen::Vector2d pixel_curr;
                // std::cout<<"point_in_c:"<<point_in_c.z()<<std::endl;
                GetPixelPosition(point_in_c, pixel_curr);

#ifdef __DEBUG__OFF
                if ((pixel_curr - data_pnp.second.second).norm() > 0)
                    std::cout << "check 2D pixel distance:" << (pixel_curr - data_pnp.second.second).norm() << "@ frame:" << frame_idx << std::endl;
#endif
                cv::circle(img, cv::Point(pixel_curr(0), pixel_curr(1)), 2,
                           cv::Scalar(0, 0, 255));
                cv::putText(img, std::to_string(mp_id), cv::Point(pixel_curr(0) + 10, pixel_curr(1) + 10), font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);

                Eigen::Vector2d pixel = data_pnp.second.second;
                cv::circle(img, cv::Point(pixel(0), pixel(1)), 2,
                           cv::Scalar(125, 125, 125));
                cv::putText(img, std::to_string(mp_id), cv::Point(pixel(0), pixel(1)), font_face, font_scale, cv::Scalar(125, 125, 125), thickness, 8, 0);
            }
            // cv::imshow("2D Viewer", img);
            // if (cv::waitKey() == 27)
            //     cv::imwrite(std::to_string(frame_idx) + ".png", img);
        }

        void GetParallelLabel(const int line_id, cv::Scalar &color_label)
        {
            bool skip = false;
            for (auto para_group : paralineid_elids_)
            {
                int vd_id = para_group.first;
                // std::cout << "vd_id:" << vd_id << std::endl;
                for (int i = 0; i < para_group.second.size(); i++)
                {
                    // std::cout << "line_id:" << para_group.second[i] << ". this line:" << line_id << std::endl;
                    if (para_group.second[i] == line_id)
                    {
                        color_label[0] = int(simulator::color_table[(vd_id + 1) % 10][0] * 255);
                        color_label[1] = int(simulator::color_table[(vd_id + 1) % 10][1] * 255);
                        color_label[2] = int(simulator::color_table[(vd_id + 1) % 10][2] * 255);
                        return;
                    }
                }
            }
        }
    };
}

#endif //VENOM_SRC_TRACK_HPP
