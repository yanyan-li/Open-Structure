/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-11-26 15:33:34
 * @LastEditTime: 2023-08-30 07:04:46
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 3D point landmarks of the environment
 * @FilePath: /JointFactorGraph/src/manager_env/EnvPoint.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_ENVPOINT_HPP__
#define __VENOM_SRC_LANDMARK_ENVPOINT_HPP__

#include "src/manager_env/EnvTrajectory.hpp" 
#include "src/utils/UtilStruct.hpp"

namespace simulator
{
    // Scalar -> Vec3  // Scalar -> cv::Matx31d
   class Trajectory; // Trajectory trajec_;

   class EnvPoint{
       public:
           // trajectory
           Trajectory* trajec_;
           // id of envpoints
           int num_id_;
           // position in the world coordinate
           Vec3 pos_world_;
           // how many Trajectorys detect this point
           int observed_;
           std::map<int/*frame_id*/, Vec3/*uvd_in_img*/> obs_frame_pixel_;
           std::map<int/*frame_id*/, Vec3/*pos_in_cam*/> obs_frame_pos_; 

           std::random_device rd;
           std::default_random_engine generator_;
           std::normal_distribution<double> pixel_n_;
           std::normal_distribution<double> depth_n_;
      
       public:
       
           EnvPoint(const int id, Trajectory* trajec):num_id_(id),observed_(1),trajec_(trajec)
           {
               double max_nt = 0.1; double max_nq = 2.0; double max_pixel_n = 1;
               std::random_device rd;
               std::default_random_engine generator(rd());
            //    std::normal_distribution<double> nt(0., max_nt);
            //    std::normal_distribution<double> nq(0., max_nq);
               std::normal_distribution<double> pixel_n(0, max_pixel_n);
               std::normal_distribution<double> depth_n(0, max_nt);
 
               generator_ = generator;
               pixel_n_ = pixel_n; 
               depth_n_ = depth_n;

               pos_world_<<-10000,-10000,-10000;
           }

           /**
            * @brief Generate ground truth envpoints for environment
            * 
            * @param distance 
            * @param axis 
            */
           void GenerateEnvPoint(const double distance, const double width, const double height, std::string axis)
           {
               std::uniform_real_distribution<double> width_point_generate( -width, width ); // width distribution
               std::uniform_real_distribution<double> height_point_generate( -height, height ); // height distribution
               std::uniform_real_distribution<double> lenghth_point_generate( distance-0.3, distance+0.2 ); // height distribution

               if(axis == "vertical-left")
                   pos_world_ <<  lenghth_point_generate(generator_), width_point_generate(generator_), height_point_generate(generator_);
               else if(axis == "vertical-right")
                   pos_world_ <<  width_point_generate(generator_),  lenghth_point_generate(generator_),  height_point_generate(generator_);
           }
           
           void GenerateEnvPoint(Eigen::Vector3d &pt)
           {
                pos_world_<< pt.x(), pt.y(), pt.z();
           }

           void GenerateEnvPoint( const Eigen::Vector3d &plane_max_bounds, const Eigen::Vector3d &plane_min_bounds)
            {
                std::uniform_real_distribution<double> width_point_generate(plane_min_bounds[0], plane_max_bounds[0]); // width distribution
                std::uniform_real_distribution<double> lenghth_point_generate(plane_min_bounds[1], plane_max_bounds[1]); // height distribution
                std::uniform_real_distribution<double> height_point_generate(plane_min_bounds[2], plane_max_bounds[2]); // height distribution
                pos_world_ <<  width_point_generate(generator_),  lenghth_point_generate(generator_),  height_point_generate(generator_);
                // std::cout<<"pos_world:"<<width_point_generate(generator_)<<","
                //          <<lenghth_point_generate(generator_)<<","
                //          <<height_point_generate(generator_)<<std::endl;
               
           }

           /**
            * @brief reproject the 3D mappoint to camera view, we than can obtain whether the mappoint can be detected or not.
            *
            * @param keyframes_Twcs
            * @param add_noise : to generate noisy mappoint
            */
           void AddObservation(std::map<int/*frame_id*/, Mat4> keyframes_Twcs, bool add_noise_to_meas)
           {   
                for(auto obs: obs_frame_pixel_ )
                    std::cout<<"check map"<<obs.first<<","<<obs.second<<std::endl;
               for(auto frame:keyframes_Twcs)
               {
                    int i = frame.first;

            //    }
            //    for (int i = 0; i < frame.second.size(); i++)
            //    {
                   // std::cout<<i<<", "<< frame.second.size()<<std::endl;
                   auto Twc = frame.second;
                   Mat4 Tcw = Twc.inverse();
                   Eigen::Matrix3d Rcw = Tcw.block(0, 0, 3, 3);
                   Vec3 tcw = Tcw.block(0, 3, 3, 1);

                   Vec3 pos_in_i;
                   double depth_in_i;
                   
                   // in the camera coordinate
                   pos_in_i = Rcw * pos_world_ + tcw;
                   depth_in_i = pos_in_i(2);

                   // condition 1: positive depth
                   if (depth_in_i <= 0.01)
                       continue; // backside of the camera
                   // condions 2: inside of the image  (x,y,1)  ---> (u-cx)/fx = x
                   double u = trajec_->cam_intri.fx * pos_in_i(0) / depth_in_i + trajec_->cam_intri.cx;
                   double v = trajec_->cam_intri.fy * pos_in_i(1) / depth_in_i + trajec_->cam_intri.cy;
                   if (u < 0 || u > trajec_->cam_intri.width || v < 0 || v > trajec_->cam_intri.height)
                       continue;
                   //std::cout<< depth_in_i<< ", "<<    trajec_->cam_intri.width <<std::endl;

                   // the i-th kf detects the env_point
                   Vec3 pixel_d_coord;
                   pixel_d_coord << u, v, depth_in_i;
                   // measured rgb-d features: (u,v,d)
                   if (add_noise_to_meas)
                   {
                    double noise_x, noise_y;
                    noise_x = pixel_n_(generator_);
                    noise_y = pixel_n_(generator_);
                    pixel_d_coord(0, 0) += noise_x;
                    pixel_d_coord(1, 0) += noise_y;
                    pixel_d_coord(2, 0) += depth_n_(generator_);
                   }
                   obs_frame_pixel_[i] = pixel_d_coord;

                   // measured 3d features in camera: (x,y,z)
                   Vec3 pos_noised_in_i;
                   double z = depth_in_i + depth_n_(generator_); // with noise
                   double x = z * (pixel_d_coord(0, 0) - trajec_->cam_intri.cx) / trajec_->cam_intri.fx;
                   double y = z * (pixel_d_coord(1, 0) - trajec_->cam_intri.cy) / trajec_->cam_intri.fy;
                   pos_noised_in_i << x, y, z;

                   // valid measurements checking
                   if (z <= 0.01)
                    continue;

#ifdef __DEBUG__OFF
                   std::cout
                       << ">>>> in the " << i << "th frame, of total " << keyframes_Twcs.size() << "frames:" << std::endl;
                   std::cout << "groundtruth uv-d:" << u <<","<<v<<","<<depth_in_i<<std::endl;
                   std::cout << "noisy uv-d:" << pixel_d_coord(0, 0) <<","<<pixel_d_coord(1, 0)<<","<<pixel_d_coord(2, 0)<<std::endl;
                   std::cout << "groundtruth XYZ in cam:" << pos_in_i(0) <<","<<pos_in_i(1)<<","<<pos_in_i(2)<<std::endl;
                   std::cout << "noisy XYZ in cam:" << x <<","<<y<<","<<z<<std::endl;
#endif
                   if (add_noise_to_meas)
                       obs_frame_pos_[i] = pos_noised_in_i; 
                   else
                       obs_frame_pos_[i] = pos_in_i;  // ground truth

                   observed_++;
               }
               
           }

           void AddObservation(std::map<int /*frame_id*/, Mat4> keyframes_Twcs, std::vector<std::pair<int /*point_id*/, int /*frame_id*/>> &associate, bool add_noise_to_meas)
           {

               for (auto asso_i : associate)
               {
                   int point_id = asso_i.first;

                   if (num_id_ == point_id)
                   {
                       int i = asso_i.second;
                       auto Twc = keyframes_Twcs[asso_i.second];
                       Mat4 Tcw = Twc.inverse();
                       Eigen::Matrix3d Rcw = Tcw.block(0, 0, 3, 3);
                       Vec3 tcw = Tcw.block(0, 3, 3, 1);

                       Vec3 pos_in_i;
                       double depth_in_i;

                       // in the camera coordinate
                       pos_in_i = Rcw * pos_world_ + tcw;
                       depth_in_i = pos_in_i(2);

                       // condition 1: positive depth
                       if (depth_in_i <= 0.01)
                           continue; // backside of the camera
                       // condions 2: inside of the image  (x,y,1)  ---> (u-cx)/fx = x
                       double u = trajec_->cam_intri.fx * pos_in_i(0) / depth_in_i + trajec_->cam_intri.cx;
                       double v = trajec_->cam_intri.fy * pos_in_i(1) / depth_in_i + trajec_->cam_intri.cy;
                       if (u < 0 || u > trajec_->cam_intri.width || v < 0 || v > trajec_->cam_intri.height)
                           continue;
                       // std::cout<< depth_in_i<< ", "<<    trajec_->cam_intri.width <<std::endl;

                       // the i-th kf detects the env_point
                       Vec3 pixel_d_coord;
                       pixel_d_coord << u, v, depth_in_i;
                       // measured rgb-d features: (u,v,d)
                       if (add_noise_to_meas)
                       {
                           double noise_x, noise_y;
                           noise_x = pixel_n_(generator_);
                           noise_y = pixel_n_(generator_);
                           pixel_d_coord(0, 0) += noise_x;
                           pixel_d_coord(1, 0) += noise_y;
                           pixel_d_coord(2, 0) += depth_n_(generator_);
                       }
                       obs_frame_pixel_[i] = pixel_d_coord;

                       // measured 3d features in camera: (x,y,z)
                       Vec3 pos_noised_in_i;
                       double z = depth_in_i + depth_n_(generator_); // with noise
                       double x = z * (pixel_d_coord(0, 0) - trajec_->cam_intri.cx) / trajec_->cam_intri.fx;
                       double y = z * (pixel_d_coord(1, 0) - trajec_->cam_intri.cy) / trajec_->cam_intri.fy;
                       pos_noised_in_i << x, y, z;

                       // valid measurements checking
                       if (z <= 0.01)
                           continue;

#ifdef __DEBUG__OFF
                       std::cout
                           << ">>>> in the " << i << "th frame, of total " << keyframes_Twcs.size() << "frames:" << std::endl;
                       std::cout << "groundtruth uv-d:" << u << "," << v << "," << depth_in_i << std::endl;
                       std::cout << "noisy uv-d:" << pixel_d_coord(0, 0) << "," << pixel_d_coord(1, 0) << "," << pixel_d_coord(2, 0) << std::endl;
                       std::cout << "groundtruth XYZ in cam:" << pos_in_i(0) << "," << pos_in_i(1) << "," << pos_in_i(2) << std::endl;
                       std::cout << "noisy XYZ in cam:" << x << "," << y << "," << z << std::endl;
#endif
                       if (add_noise_to_meas)
                           obs_frame_pos_[i] = pos_noised_in_i;
                       else
                           obs_frame_pos_[i] = pos_in_i; // ground truth

                       observed_++;
                   }
               }
           }
           void print()
           {
               std::cout<<"\033[0;31m [Venom Similator Printer] The global position of mappoint is  \033[0m"<<std::endl
                        <<pos_world_<<std::endl<< "which is detected by "<<observed_<<"cameras"<<std::endl;
           }


           
   };
}

#endif // __VENOM_SRC_LANDMARK_ENVPOINT_HPP__
