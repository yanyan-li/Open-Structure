/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-30 14:02:44
 * @LastEditTime: 2023-08-19 07:34:18
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: Venom 
 * @FilePath: /JointFactorGraph/src/map/MapVenom.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPVENOM_HPP_
#define __VENOM_SRC_LANDMARK_MAPVENOM_HPP_

#include "../manager_env/EnvTrajectory.hpp"
#include <map>
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
 
#include <eigen3/Eigen/Dense>
namespace simulator
{
    static int vm_id =0;
    class Trajectory;

    class MapVenom{  //MapVenom should save venom under World Coordinate
        public:
            Trajectory * ptr_traject_;
            int num_id_;
            int observed_;
            int anchor_frame_id_;// the frame that detects this MapVenom fristly.
            Eigen::Matrix4d anchor_frame_pose_; // camera pose of Anchor Frame 
            int venom_type_;
            Eigen::Matrix3d rotation_venom_world_; 
            
            // this is the shared normlized vanishing direction \in S^2.
            Eigen::Vector2d parali_vd_world_;
            // this is the 2-DoF parameters for each line in ParaLine.
            std::vector< std::pair<int/*landmark_id*/, Eigen::Vector2d/*two parameters*/ >> parali_theta_gammar_world_; 
            
            // mapplines of this venom
            std::vector<std::pair<int, Eigen::Matrix<double,3,2>>> related_lines_world_;

            //Trajectory* traject_;
            std::vector</*frame_id*/ std::pair<int/*landmark_id*/, Eigen::Vector4d/*paraline_in_cam*/>> vec_obs_;  // noised observation
            std::vector</*frame_id*/ std::pair<int/*landmark_id*/, Eigen::Vector4d/*paraline_in_cam*/>> vec_obs_gt_;  // noised observation


        public:
            MapVenom(Trajectory* traject, int anchor_frame_id, int type, std::vector<std::pair<int/*landmark_id*/, Eigen::Matrix<double,3,2> >> &vec_parallel_mls)
                    :ptr_traject_(traject), anchor_frame_id_(anchor_frame_id),venom_type_(type), related_lines_world_(vec_parallel_mls),
                     anchor_frame_pose_(Eigen::Matrix4d::Zero())
            {
                num_id_ = vm_id++;

                // initialize with ParaLine
                anchor_frame_pose_ = ptr_traject_->groundtruth_traject_id_Twc_[anchor_frame_id]; 


                // in our parameterization
                RepresentInParaLine();


            };

            //
            void RepresentInParaLine()
            {
                //--> initialization ParaLine Parameterization
                Eigen::Vector3d vanishing_direction = Eigen::Vector3d::Zero();

                // 
                std::map<int/*landmark_id*/, double/*beta*/> set_betas;
                
                //
                std::map<int, Eigen::Vector3d> set_normals; 

                for(size_t i=0; i< related_lines_world_.size(); i++)
                {
                    Eigen::Vector3d direction = related_lines_world_[i].second.block(0,0,3,1) - related_lines_world_[i].second.block(0,1,3,1);

                    // beta
                    double weight2 = direction.norm();
                    double beta = asin(weight2); 
                    set_betas[i] = beta; 

                    // normal vector
                    Eigen::Vector3d ray_start_center = related_lines_world_[i].second.block(0,0,3,1)-anchor_frame_pose_.block(0,3,3,1);
                    Eigen::Vector3d ray_end_center = related_lines_world_[i].second.block(0,1,3,1)-anchor_frame_pose_.block(0,3,3,1);
                    Eigen::Vector3d normal = ray_start_center.cross(ray_end_center);

                    double weight1 = normal.norm();
                    assert( abs(asin(weight2)-acos(weight1))<0.02 );
                    set_normals[i] = normal.normalized();//.normalize();

                    vanishing_direction += direction;
                }

                // normalized vector
                vanishing_direction = vanishing_direction.normalized(); //.normalize();   

                //step1: theta and varphi
                parali_vd_world_(0) = acos(vanishing_direction(2));
                // TODO: sin(theta)=0
                assert(sin(parali_vd_world_(0))!=0); 
                parali_vd_world_(1) = acos(vanishing_direction(0)/sin(parali_vd_world_(0)));

                //step2: alpha1
                for (size_t i = 0; i < related_lines_world_.size(); i++)
                {
                    // beta1 sin(beta)
                    // double sin_theta =  sin(parali_vd_world_(0));
                    double cos_theta =  cos(parali_vd_world_(0));
                    double sin_varphi = sin(parali_vd_world_(1)); 
                    double cos_varphi = cos(parali_vd_world_(1));

                    
                    double alpha = acos(set_normals[i](0)/(-sin_varphi-cos_theta*cos_varphi));
                    
                    Eigen::Vector2d beta_alpha( set_betas[i],  alpha);
                    parali_theta_gammar_world_.push_back( std::make_pair(i,beta_alpha ));
                    
                }
                
            }

            void AddObservation(int venom_frame_id, Eigen::Vector4d rotation_cam_venom)
            {
                vec_obs_.push_back(std::make_pair(venom_frame_id, rotation_cam_venom));
            }



    };



}




#endif //__VENOM_SRC_LANDMARK_MAPVENOM_HPP_