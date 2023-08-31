/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-09-17 16:48:58
 * @LastEditTime: 2023-08-19 07:26:06
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /JointFactorGraph/src/map/MapPoint.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_MAPPOINT_HPP__
#define __VENOM_SRC_LANDMARK_MAPPOINT_HPP__

#include "../manager_env/EnvTrajectory.hpp" 

namespace simulator
{
    // Scalar -> Eigen::Vector3d  // Scalar -> cv::Matx31d
   class Trajectory; // Trajectory trajec_;

   class MapPoint{
       public:
           // id of the mappoint
           int num_id_;
           // how many Trajectorys detect this point
           int observed;
           // position in the world coordinate
           Eigen::Vector3d pos_world_;
           Eigen::Vector3d pos_world_noise_;
           Trajectory* trajec_;
           /**
            * @brief The mappoint is detected by the i_Trajectory ^{th} Trajectory,
            *        and associated to the i_pixel ^{th} pixel. 
            */
           std::map<int /*id_poxe*/, int /*i_pixel*/> observations_;
           std::vector< std::pair<int, Eigen::Vector3d>> obs;  // noised observation
           std::vector< std::pair<int, Eigen::Vector3d>> obs_gt; // gt observation
          
           std::random_device rd;
           std::default_random_engine generator_;
           std::normal_distribution<double> pixel_n_;
      
       public:
       
           MapPoint(const int id, Trajectory* trajec):num_id_(id),observed(0),trajec_(trajec)
           {
               double max_nt = 0.1; double max_nq = 1.*M_PI/180.; double max_pixel_n = 1./240;
               std::random_device rd;
               std::default_random_engine generator(rd());
               std::normal_distribution<double> nt(0., max_nt);
               std::normal_distribution<double> nq(0., max_nq);
               std::normal_distribution<double> pixel_n(0, max_pixel_n);
 
               generator_ = generator;
               pixel_n_ = pixel_n; 

               // initialize 
               obs.reserve(trajec_->groundtruth_traject_id_Twc_.size());
               obs_gt.reserve(trajec_->groundtruth_traject_id_Twc_.size());
           }

           /**
            * @brief Generate ground truth mappoints for environment
            * 
            * @param distance 
            * @param axis 
            */
           void GenerateMapPoint(const double distance, std::string axis)
           {
              
               std::uniform_real_distribution<double> point_generate( -3., 2. ); // width distribution
               if(axis == "vertical-left")
                   pos_world_ <<  distance, point_generate(generator_), point_generate(generator_);
               else if(axis == "vertical-right")
                   pos_world_ <<  point_generate(generator_), distance,  point_generate(generator_);

           }
          
           /**
            * @brief reproject the 3D mappoint to camera view, we than can obtain whether the mappoint can be detected or not.
            *
            * @param keyframes_Twcs
            * @param add_noise : to generate noisy mappoint
            */
           void AddObservation(std::vector<Eigen::Matrix4d> keyframes_Twcs, bool add_noise)
           {
               for(size_t i = 0; i< keyframes_Twcs.size(); ++i)
               {
                   //std::cout<<i<<", "<< keyframes_Twcs.size()<<std::endl;
                   auto Twc = keyframes_Twcs[i];
                   Eigen::Matrix4d Tcw = Twc.inverse();
                   Eigen::Matrix3d Rcw = Tcw.block(0,0,3,3);
                   Eigen::Vector3d tcw = Tcw.block(0,3,3,1);
 
                   Eigen::Vector3d ob_in_i;
                   double depth_in_i;
 
                   // in the camera coordinate
                   ob_in_i = Rcw * pos_world_ + tcw;
                   depth_in_i = ob_in_i(2);

                   //condition 1: positive depth
                   if(depth_in_i <= 0.1) continue; // backside of the camera
                   
                   // condions 2: inside of the image
                   double u = trajec_->cam_intri.fx* ob_in_i(0)/depth_in_i+ trajec_->cam_intri.cx; 
                   double v = trajec_->cam_intri.fy* ob_in_i(1)/depth_in_i+ trajec_->cam_intri.cy;
                   if(u<0 || u>trajec_->cam_intri.width || v<0 || v>trajec_->cam_intri.height)
                    continue;

                   // double associate 
                   // A. frame detects the "num_id, ob_in_i"
                   //trajec_->SetKeyFrameDetects(i, this->num_id_, ob_in_i);
                   observed++;
                   // B. this mappoint's observation vector includs <i, ob_in_i)
                   obs_gt.emplace_back(i, ob_in_i);
                   // C. noisy 
                   if(add_noise && obs_gt.size() > 1)
                   //            if(add_noise )
                   {
                       Eigen::Vector3d noise ( pixel_n_(generator_),pixel_n_(generator_), 0 );
                       ob_in_i += noise;
                   }
                   obs.emplace_back(i, ob_in_i);
               }
 
           }
 
           void print()
           {
               std::cout<<"\033[0;31m [Venom Similator Printer] The global position of mappoint is  \033[0m"<<std::endl
                        <<pos_world_<<std::endl<< "which is detected by "<<observed<<"cameras"<<std::endl;
 
           }
   };
}

#endif // __VENOM_SRC_LANDMARK_MAPPOINT_HPP__