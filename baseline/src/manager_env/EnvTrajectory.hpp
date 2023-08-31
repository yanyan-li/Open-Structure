/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 15:22:39
 * @LastEditTime: 2023-08-29 12:47:01
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: This class is to generate different types of camera poses.
 * @FilePath: /JointFactorGraph/src/manager_env/EnvTrajectory.hpp
 */
#ifndef __VENOM_SRC_TRAJECTORY_HPP__
#define __VENOM_SRC_TRAJECTORY_HPP__ 

#include <map>
#include <opencv2/core/persistence.hpp>
#include <vector>
#include <iostream>
#include <random>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "EnvFrame.hpp" 
#include "src/utils/UtilEnvElement.hpp"

namespace simulator
{

    class Trajectory
    {
    public:
        
        enum TrajectoryType
        {
            CYCLE = 0,
            SPHERE = 1,
            CORRIDOR = 2,
            PUBLIC = 9
            // RECTANGLE = 2,
            // HEXAGON = 3,
            // LINE = 4
        };

        struct CamIntrinsic
        {
            float fx;
            float fy ;
            float cx ;
            float cy;
            int width;
            int height;

        }cam_intri;

        struct TrackParameters
        {
            bool cam_observ_noise; 
        }track_para;

        /**
         * @brief Initialization
         * 
         * @param type 
         * @param num_keyframe 
         */
        Trajectory( cv::FileStorage &settings )
        {   
            settings["Camera1.fx"] >> cam_intri.fx; 
            settings["Camera1.fy"] >> cam_intri.fy; 
            settings["Camera1.cx"] >> cam_intri.cx; 
            settings["Camera1.cy"] >> cam_intri.cy;
            
            settings["Camera.width"] >> cam_intri.width;
            settings["Camera.height"] >> cam_intri.height;
            settings["Env.Public.envtraj_path"] >> public_traj_path_;
            return; 
        };

        Trajectory(const int type, const int num_keyframe, cv::FileStorage &settings )
        {   
            settings["Camera1.fx"] >> cam_intri.fx; 
            settings["Camera1.fy"] >> cam_intri.fy; 
            settings["Camera1.cx"] >> cam_intri.cx; 
            settings["Camera1.cy"] >> cam_intri.cy;
            
            settings["Camera.width"] >> cam_intri.width;
            settings["Camera.height"] >> cam_intri.height;
            settings["Env.Public.envtraj_path"] >> public_traj_path_;

#ifdef __VERBOSE__
            std::cout << "cam_fx: " << cam_intri.fx << ", " << cam_intri.fy << std::endl;
#endif 

            if(type==0)
            {
                traject_type_=CYCLE;
                std::cout<<"\033[0;35m[Venom Similator Printer]The Cycle trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }
            else if(type==1)
            {
                traject_type_=SPHERE;
                std::cout<<"\033[0;35m[Venom Similator Printer]The Sphere trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }
            else if(type==2)
            {
                traject_type_=CORRIDOR;
                std::cout<<"\033[0;35m[Venom Similator Printer]The Corridor trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }else if(type==9)
            {
                traject_type_=PUBLIC;
                std::cout<<"\033[0;35m[Venom Similator Printer]The PubSeq trajectory with "<< num_keyframe <<" frames is generated.\033[0m"<< std::endl; 
            }
                return; 
        };

        
        bool GenerateTrajectory(const TrajectoryType &traject_type, const int &num_keyframe, const double &r)
        {
            if (num_keyframe < 2)
                return false;
            
            //double r = 3.0;

            if (traject_type == CYCLE)
            {
                int wave_num = 10;
                double wave_high = 1.5;
                GenerateCycleKeyFrames(num_keyframe, wave_num, wave_high, 9);
                double length = 8.0, width = 7, height = 5;
                Objects objects_env = Objects(length, width, height);
                objects_env.GetRegions(cube_top_planes_, cube_right_planes_,
                                       cube_bottom_planes_, cube_left_planes_, box_4_planes_);
            }
            else if (traject_type == SPHERE)
            {
                GenerateSphereKeyFrames(num_keyframe, r);
                double length = 8.0, width = 7, height = 5;
                Room room_env = Room(length, width, height);
                room_env.GetRegions(cube_top_planes_, cube_right_planes_,
                                    cube_bottom_planes_, cube_left_planes_);
            }
            else if (traject_type == CORRIDOR)
            {
                // double r = 1.0;
                double length = 2*r;
                double height= 3.5;
                GenerateInDoorFrames(num_keyframe, r, length);

                // double length=28.0, width =27, height=3.5;
                Corridor corridor_env = Corridor(2*(length+r+2), 2*(length+r+2), height);
                // corridor_env.GetRegions(cube_top_planes_, cube_left_planes_);
                corridor_env.GetRegions(cube_top_planes_, cube_right_planes_, 
                                        cube_bottom_planes_, cube_left_planes_);
            }
            else if (traject_type == PUBLIC)
            {
                GeneratePublicKeyFrames(public_traj_path_);
            }
            else
            {
                std::cout << "\033[0;35m [Venom Similator Printer] We only have the four types of trajectory currently.\033[0m" << std::endl;
                return false;
            }
            // vectors
            //std::cout<<"trajec 2:"<<num_keyframe<< std::endl;
            // InitPoseVector(num_keyframe);
            return true;
        } 

        void GeneratePublicKeyFrames(const std::string &traj_path) //,  std::vector<Eigen::Matrix4d> &vec_traj)
        {
            std::cout << "\033[0;33m[Venom Simulator Printer] Saving keyframe trajectory to " << traj_path << ".\033[0m" << std::endl;
            
            std::ifstream file(traj_path);
            std::string line;
            int frame_idx = 0;
            while (std::getline(file, line))
            {
                std::vector<std::string> words;
                boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
                //std::cout<<words[0]<<std::endl;
               
                Eigen::Vector3d trans = Eigen::Vector3d::Zero();
                Eigen::Quaterniond rot;
                double v_index = boost::lexical_cast<double>(words[0]);
                //std::cout<<v_index<<std::endl;

                trans.x() = boost::lexical_cast<double>(words[1]);
                trans.y() = boost::lexical_cast<double>(words[2]);
                trans.z() = boost::lexical_cast<double>(words[3]);

                rot.x() = boost::lexical_cast<double>(words[4]);
                rot.y() = boost::lexical_cast<double>(words[5]);
                rot.z() = boost::lexical_cast<double>(words[6]);
                rot.w() = boost::lexical_cast<double>(words[7]);
                
                Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
                Eigen::Matrix3d R = Eigen::Quaterniond(rot.w(), rot.x(), rot.y(), rot.z()).toRotationMatrix();
                pose.template block<3, 3>(0, 0) = R;
                pose.template block<3, 1>(0, 3) = trans;

                // vec_traject_gt_Twc_.push_back(pose);
                groundtruth_traject_id_Twc_[frame_idx] = pose;
                frame_idx ++;
    
            }
            std::cout<<"groundtruth_traject_id_Twc_,"<<groundtruth_traject_id_Twc_.size()<<std::endl;
            file.close();
        }

        void CyclePoseGeneration(double theta, double wave_theta, double step, int radius, double wave_high, int n)
        {
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            double theta_ = 0;
            if (theta < M_PI / 2.)
                theta_ = theta + M_PI / 2.;
            else if (theta >= M_PI / 2. && theta < M_PI)
                theta_ = theta - M_PI * 3. / 2.;
            else if (theta >= M_PI && theta < M_PI * 3. / 2.)
                theta_ = theta + M_PI / 2.;
            else if (theta >= M_PI * 3. / 2. && theta < M_PI * 2)
                theta_ = theta - 3. * M_PI / 2.;
            Eigen::Matrix3d Rwb;
            Rwb = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-M_PI/2.-M_PI/6., Eigen::Vector3d::UnitX());
            Eigen::Vector3d twb = Eigen::Vector3d(radius * cos(theta), radius * sin(theta), wave_high * std::cos(wave_theta));

            Twc.block(0, 0, 3, 3) = Rwb;
            Twc.block(0, 3, 3, 1) = twb;
            // std::cout<<"Twc:"<<Twc<<std::endl;
            // vec_traject_gt_Twc_.push_back(Twc);
            groundtruth_traject_id_Twc_[n] = Twc;
            // std::cout<<"finished"<<","<<vec_traject_gt_Twc_.size()<<std::endl;
        }
        
        void SpherePoseGeneration(double radius, int nums_points, int index)
        {
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            double phi = acos(-1.0 + (2.0 * (index + 1) - 1.0) / nums_points);
            double theta = sqrt(nums_points * M_PI) * phi;
            double t_x = radius * cos(theta) * sin(phi);
            double t_y = radius * sin(theta) * sin(phi);
            double t_z = radius * cos(phi);
            Eigen::Vector3d twb;
            twb << t_x, t_y, t_z;

            Eigen::Matrix3d Rwb;
            Rwb = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitX());

            Twc.block(0, 0, 3, 3) = Rwb;
            Twc.block(0, 3, 3, 1) = twb;

            // vec_traject_gt_Twc_.push_back(Twc);
            groundtruth_traject_id_Twc_[index] = Twc;
        }

        bool GenerateCycleKeyFrames(const int frame_num, int wave_num, double wave_high, double radius_cycle = 0)
        {
            if (frame_num < 2)
                return false;
            std::cout<<"Generate Cycle KeyFrames"<<std::endl;
            for (int n = 0; n < frame_num; n++) // camera id from n=0 to n=frameNum-1
            {
                int radius = radius_cycle;
                //double wave_high = 0.75;
                double theta = n * 2. * M_PI / frame_num;
                double step = n * 1. / frame_num;
                double wave_theta = n * 2. * M_PI / (frame_num / wave_num);

                // std::cout<<"n:"<<n<<","<<theta <<","<<step<<","<<wave_high<<std::endl;
                CyclePoseGeneration(theta, wave_theta, step, radius, wave_high, n);
            }
        }
        
        bool GenerateSphereKeyFrames(const int frame_num, double radius)
        {
            if (frame_num < 2)
                return false;
            for (int n = 0; n < frame_num; n++)
            {
                double radius = 2.0;
                // double theta = n * 2. * M_PI / frame_num;
                SpherePoseGeneration(radius, frame_num, n);
            }
        }
        
        bool GenerateInDoorFrames(const int frame_num, double radius, double length_half)
        {
            if (frame_num < 8)
                return false;
            int part_num = frame_num / 8;
            for (int n = 0; n < frame_num; n++)
            {
                Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                int batch_type = n / part_num;
                int batch_id = n % part_num;
                double batch_dx = (double)batch_id / (double)part_num;
                if(batch_type%2 == 0)
                {
                    if(batch_type == 0){
                        Eigen::Vector3d twb;
                        twb << length_half + radius, (-length_half + batch_dx * 2 * length_half), 0;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 2){
                        Eigen::Vector3d twb;
                        twb << (length_half - batch_dx * 2 * length_half), length_half + radius, 0;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 4){
                        Eigen::Vector3d twb;
                        twb << -(length_half + radius), (length_half - batch_dx * 2 * length_half), 0;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 6){
                        Eigen::Vector3d twb;
                        twb << (-length_half + batch_dx * 2 * length_half), -(length_half + radius), 0;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(3* M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                }
                else {
                    if(batch_type == 1){
                        Eigen::Vector3d twb;
                        double radius_x = std::cos(batch_dx * M_PI * 0.5 ) * radius;
                        double radius_y = std::sin(batch_dx * M_PI * 0.5 ) * radius;
                        twb << length_half, length_half, 0;
                        twb(0) += radius_x;
                        twb(1) += radius_y;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(batch_dx * M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 3){
                        Eigen::Vector3d twb;
                        double radius_x = std::cos(batch_dx * M_PI * 0.5 ) * radius;
                        double radius_y = std::sin(batch_dx * M_PI * 0.5 ) * radius;
                        twb << - length_half, length_half, 0;
                        twb(0) -= radius_y;
                        twb(1) += radius_x;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(batch_dx * M_PI/2 + M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 5){
                        Eigen::Vector3d twb;
                        double radius_x = std::cos(batch_dx * M_PI * 0.5 ) * radius;
                        double radius_y = std::sin(batch_dx * M_PI * 0.5 ) * radius;
                        twb << - length_half, - length_half, 0;
                        twb(0) -= radius_x;
                        twb(1) -= radius_y;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(batch_dx * M_PI/2 + M_PI, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                    else if(batch_type == 7){
                        Eigen::Vector3d twb;
                        double radius_x = std::cos(batch_dx * M_PI * 0.5 ) * radius;
                        double radius_y = std::sin(batch_dx * M_PI * 0.5 ) * radius;
                        twb << length_half, - length_half, 0;
                        twb(0) += radius_y;
                        twb(1) -= radius_x;

                        Eigen::Matrix3d Rwb;
                        Rwb = Eigen::AngleAxisd(batch_dx * M_PI/2 +3* M_PI/2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

                        Twc.block(0, 0, 3, 3) = Rwb;
                        Twc.block(0, 3, 3, 1) = twb;
                    }
                
                }

                if (batch_type >7) continue;

                // vec_traject_gt_Twc_.push_back(Twc);
                groundtruth_traject_id_Twc_[n] = Twc;
            }

        }

        void PrintTrajectory()
        {
            for (auto id_pose: groundtruth_traject_id_Twc_)
            {
                std::cout << "\033[0;35m This is the\033[0m" << id_pose.first << " th camera: " << std::endl
                          << id_pose.second << std::endl;
            }
        }

    public:
        int num_id_; // Trajectory id
        int n_;      // landmarks detected by the Trajectory
        // std::vector<Eigen::Matrix4d> vec_traject_gt_Twc_;
        std::map<int, Eigen::Matrix4d> groundtruth_traject_id_Twc_;
        // std::vector<Eigen::Matrix4d> vec_traject_vo_Twc_;
        // std::vector<Eigen::Matrix4d> vec_traject_vo_opti_Twc_;
        std::map<std::string, std::vector<Eigen::Vector3d>> cube_top_planes_;
        std::map<std::string, std::vector<Eigen::Vector3d>> cube_left_planes_; 
        std::map<std::string, std::vector<Eigen::Vector3d>> cube_bottom_planes_;
        std::map<std::string, std::vector<Eigen::Vector3d>> cube_right_planes_;
        std::map<std::string, std::vector<Eigen::Vector3d>> box_4_planes_;

        // std::vector<std::pair<int /* */, int> >
        TrajectoryType traject_type_;
        // key: frame_id; value: primitive_nums
        // std::map<int, int> contain_mp_cams_;
        // std::map<int, int> contain_ml_cams_;
        // std::map<int /*frame_id*/, int /*size_of_venom */> contain_mw_cams_;
        std::string public_traj_path_;

    protected:
        Eigen::Vector3d pos_normalized_Trajectory;
        // std::map<int /*pos_world_id*/,int /*measurements_id*/> associations;
    };
}

#endif // VENOM_SRC_TRAJECTORY_HPP__