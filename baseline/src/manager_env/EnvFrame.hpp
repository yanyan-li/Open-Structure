/*** 
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-11-16 15:20:40
 * @LastEditTime: 2023-07-29 13:36:53
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /JointFactorGraph/src/manager_env/EnvFrame.hpp
 */
#ifndef __VENOM_SRC_MANAGER_FRAME_HPP__
#define __VENOM_SRC_MANAGER_FRAME_HPP__


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../utils/UtilStruct.hpp"

namespace simulator
{
    struct ElementInfo
        {
            int feature_id_;
            Eigen::Vector2d feature_;
            Eigen::Vector3d mappoint_in_cam_;
            Eigen::Vector3d mappoint_in_world_; 
        
        };
    
    class Frame
    {
        
        public:
            
            std::vector<simulator::ElementInfo> vec_elements_;
            
            std::vector<std::pair<int/*feature_id*/, Eigen::Vector2d>> vec_pixels_;
            std::vector<std::pair<int/*feature_id*/, Eigen::Vector3d>> vec_mappoints_; 
            Eigen::Matrix4d Twc_estimated_;
            int width;
            int height;

            int frame_id_;
            cv::Mat img;

            Frame(int id, cv::FileStorage &settings) : frame_id_(id)
            {
                // parameters
                settings["Camera.width"] >> width;
                settings["Camera.height"] >> height;
                // std::cout<<"width:"<<width<<","<<height<<std::endl;
                // init image

                // img = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));


                // save image
                // BuildBlackImage(std::to_string(frame_id_));
                // DrawFeatures();
            }

            void BuildBlackImage(const std::string &file_name)
            {
                //
                cv::imwrite(file_name + ".png", img);
            }

            void DrawFeatures(simulator::FrameFeatures &features)
             {
            //     for (auto mit = features.pixels.begin(); mit != features.pixels.end(); mit++)
            //     {
            //         int mappoint_id = mit->first;
            //         Vec2 pixel = mit->second;
            //         // TODO: draw points and record pixel_id
            //     }

            //     for (auto mit = features.endpoints.begin(); mit != features.endpoints.end(); mit++)
            //     {
            //         int mapline_id = mit->first;
            //         Mat2 endpoints = mit->second;

            //         Vec2 pixel_u = endpoints.col(0);
            //         Vec2 pixel_v = endpoints.col(1);

            //         // TODO: draw points and record pixel_id

            //         cv::line(img, cv::Point(pixel_u(0), pixel_u(1)),
            //                  cv::Point(pixel_v(0), pixel_v(1)),
            //                  cv::Scalar(125, 155, 55));
            //     }
            //     BuildBlackImage("feat_" + std::to_string(frame_id_));
            }
    };
}


#endif //__VENOM_SRC_MANAGER_FRAME_HPP__