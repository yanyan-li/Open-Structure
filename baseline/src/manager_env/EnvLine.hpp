/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-11-26 15:33:48
 * @LastEditTime: 2023-01-31 16:38:06
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /VENOM/src/landmark/EnvLine.hpp
 */

#ifndef __VENOM_SRC_LANDMARK_ENVLINE_HPP__
#define __VENOM_SRC_LANDMARK_ENVLINE_HPP__

#include "src/manager_env/EnvTrajectory.hpp" 
#include "src/utils/UtilStruct.hpp"

namespace simulator
{
// Scalar -> Vec3  // Scalar -> cv::Matx31d
class Trajectory; // Trajectory traject_;

class EnvLine
{
  public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // trajectory
    Trajectory *traject_;
    // id of the mappoint
    int num_id_;
    // endpoints parametrization in the world coordinate
    Mat32 pos_world_;
    // how many Trajectorys detect this point
    int observed_;

    /**
     * @brief The env_point is detected by the i_Trajectory ^{th} Trajectory,
     *        and associated to the i_pixel ^{th} pixel.
     */
    std::map<int /*frame_id*/, Mat32> obs_frameid_linepixel_;
    std::map<int /*frame_id*/, Mat32 /*pos_in_cam*/> obs_frameid_linepos_;

    std::random_device rd;
    std::default_random_engine generator_;
    std::normal_distribution<double> pixel_n_;
    std::normal_distribution<double> depth_n_;

    // type: 0, 1, 2
    int vanishing_direction_type_;
    Vec3 vanishing_direction_;
    //

    //////////////////////////////////////

    Mat32 pos_world_noise_;
    // plucker parametrization in the world coordinate
    Mat32 plucker_world_;
    Mat32 plucker_anchor_noise_;
    // anchor frame （since we need to compute the normal vector for plucker parameter）
    /**
     * @brief The mappoint is detected by the i_Trajectory ^{th} Trajectory,
     *        and associated to the i_pixel ^{th} pixel.
     */
    std::map<int /*id_poxe*/, int /*i_pixel*/> observations_;
    std::vector</*frame_id*/ std::pair<int /*landmark_id*/, Mat32 /*posi_in_cam*/>> vec_obs_; // noised observation
    std::vector<std::pair<int, Mat32>> vec_obs_gt_;                                           // gt observation

  private:
    double start_depth_in_cam;
    double end_depth_in_cam;
    int anchor_frame_id_;
    Mat4 T_w_anchor_ = Mat4::Zero();
    std::pair<int /*frame_id*/, Mat4 /*frame_pose*/> anchor_paras_;
    double length = 0;

  public:
    EnvLine(const int id, Trajectory *traject) : num_id_(id), observed_(1), traject_(traject), anchor_frame_id_(-1)
    {
        // noise
        double max_nt = 0.05;
        double max_nq = 2.0;
        double max_pixel_n = 1;

        std::random_device rd;
        std::default_random_engine generator(rd());
        // std::normal_distribution<double> nt(0., max_nt);
        // std::normal_distribution<double> nq(0., max_nq);
        std::normal_distribution<double> pixel_n(0, max_pixel_n);
        std::normal_distribution<double> depth_n(0, max_nt);

        // std::normal_distribution<double> pixel_n(1, max_pixel_n);
        pixel_n_ = pixel_n;
        generator_ = generator;
        depth_n_ = depth_n;
    };


    void Reproject(Mat32 &ob_gt, Eigen::Matrix2d &pixel_gt)
    {
        Vec3 start_point = ob_gt.block(0, 0, 3, 1);
        Vec3 end_point = ob_gt.block(0, 1, 3, 1);

        // (u-cx)/fx = x
        double fx = traject_->cam_intri.fx;
        double fy = traject_->cam_intri.fy;
        double cx = traject_->cam_intri.cx;
        double cy = traject_->cam_intri.cy;

        double start_u = fx * start_point(0) / start_point(2) + cx;
        double start_v = fy * start_point(1) / start_point(2) + cy;

        double end_u = fx * end_point(0) / end_point(2) + cx;
        double end_v = fy * end_point(1) / end_point(2) + cy;

        pixel_gt(0, 0) = start_u;
        pixel_gt(0, 1) = end_u;
        pixel_gt(1, 0) = start_v;
        pixel_gt(1, 1) = end_v;
    }

    void ProjectWithNoise(Eigen::Matrix2d &measurements, Mat32 &ob_means)
    {

        double fx = traject_->cam_intri.fx;
        double fy = traject_->cam_intri.fy;
        double cx = traject_->cam_intri.cx;
        double cy = traject_->cam_intri.cy;

        double start_x = start_depth_in_cam * (measurements(0, 0) - cx) / fx;
        double start_y = start_depth_in_cam * (measurements(1, 0) - cy) / fy;

        double end_x = end_depth_in_cam * (measurements(0, 1) - cx) / fx;
        double end_y = end_depth_in_cam * (measurements(1, 1) - cy) / fy;

        ob_means.block(0, 0, 3, 1) << start_x, start_y, start_depth_in_cam;
        ob_means.block(0, 1, 3, 1) << end_x, end_y, end_depth_in_cam;
    }

    void GenerateEnvLine(const double distance, const double width, const double height, std::string axis)
    {
        std::uniform_real_distribution<double> point_generate(-width, width); // width distribution
        std::uniform_real_distribution<double> distance_generate(distance - 0.1, distance + 0.1);
        if (axis == "vertical-left")
        {
            pos_world_(0, 0) = distance_generate(generator_);
            pos_world_(0, 1) = pos_world_(0, 0);
            pos_world_(1, 0) = point_generate(generator_);
            pos_world_(1, 1) = pos_world_(1, 0);
            pos_world_(2, 0) = height;
            pos_world_(2, 1) = -height;

            vanishing_direction_type_ = 0;
            vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
            // std::cout<<" vanishing_direction: "<<vanishing_direction_<<std::endl;
        }
        else if (axis == "vertical-right")
        {
            pos_world_(0, 0) = point_generate(generator_);
            pos_world_(0, 1) = pos_world_(0, 0);
            pos_world_(1, 0) = distance_generate(generator_);
            pos_world_(1, 1) = pos_world_(1, 0);
            pos_world_(2, 0) = height;
            pos_world_(2, 1) = -height;
            vanishing_direction_type_ = 0;
            vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
            // std::cout<<" vanishing_direction: "<<vanishing_direction_<<std::endl;
        }
        else if (axis == "horizontal-left")
        {
            // x ,y, z
            pos_world_(0, 0) = point_generate(generator_);
            pos_world_(0, 1) = pos_world_(0, 0);
            pos_world_(1, 0) = -height;
            pos_world_(1, 1) = height;
            pos_world_(2, 0) = -3.2;
            pos_world_(2, 1) = -3.2;
            vanishing_direction_type_ = 1;
            vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
            // std::cout<<" vanishing_direction: "<<vanishing_direction_<<std::endl;
        }
        else if (axis == "horizontal-right")
        {
            // x ,y, z
            pos_world_(0, 0) = -height;
            pos_world_(0, 1) = height;
            pos_world_(1, 0) = point_generate(generator_);
            pos_world_(1, 1) = pos_world_(1, 0);
            pos_world_(2, 0) = -3.2;
            pos_world_(2, 1) = -3.2;
            vanishing_direction_type_ = 2;
            vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
            // std::cout<<" vanishing_direction: "<<vanishing_direction_<<std::endl;
        }

    }

    void GenerateEnvLine(const Eigen::Vector3d &plane_max_bounds, 
                     const Eigen::Vector3d &plane_min_bounds)
    {
        std::random_device rd;
        std::default_random_engine generator(rd());
        
        int dominent_direct = -1;
        int plane_direc = -1;
        double max_distance = -10000.0;
        double min_distance = 10000.0;
        std::map<int, double> ep_value;
        std::map<int, double> sp_value;
        for(int i=0; i<3; i++)
        {
            double i_aix =  std::abs(plane_max_bounds[i] - plane_min_bounds[i]);
            if(i_aix>max_distance)
            {
                dominent_direct = i;
                max_distance = i_aix;
            }
            if(i_aix<min_distance)
            {
                plane_direc = i;
                min_distance = i_aix;
            }
        }
        // std::cout<<"check:"<<dominent_direct<<","<<plane_direc<<std::endl;
        
        for(int i=0; i<3;i++)
        {
            if(i==dominent_direct || i==plane_direc )
                continue;
            
            std::uniform_real_distribution<double> support_point_generate(plane_min_bounds[i], plane_max_bounds[i]); // width distribution
            double support =  support_point_generate(generator);

            ep_value[i] = support;
            sp_value[i] = support;
        }

        std::uniform_real_distribution<double> dominament_sp_generate(plane_min_bounds[dominent_direct], plane_max_bounds[dominent_direct]); // width distribution
        std::uniform_real_distribution<double> dominament_ep_generate(plane_min_bounds[dominent_direct], plane_max_bounds[dominent_direct]); // width distribution
        std::uniform_real_distribution<double> plane_point_generate(plane_min_bounds[plane_direc], plane_max_bounds[plane_direc]); // width distribution
        double sp =  dominament_sp_generate(generator);
        double ep =  dominament_ep_generate(generator);
        ep_value[dominent_direct] = ep;
        sp_value[dominent_direct] = sp;
        
        double plane_p =  plane_point_generate(generator);
        ep_value[plane_direc] = plane_p;
        sp_value[plane_direc] = plane_p;

        pos_world_.col(0)<<  sp_value[0],  sp_value[1],  sp_value[2];
        pos_world_.col(1)<<  ep_value[0],  ep_value[1],  ep_value[2];
        
        vanishing_direction_type_ = dominent_direct;
        vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();

        // make sure unified vanishing directions
        if(vanishing_direction_[dominent_direct]<0)
        {
            pos_world_.col(1)<<  sp_value[0],  sp_value[1],  sp_value[2];
            pos_world_.col(0)<<  ep_value[0],  ep_value[1],  ep_value[2];
            
            vanishing_direction_type_ = dominent_direct;
            vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
        }
    }

    void GenerateEnvLine(Eigen::Matrix<double, 7, 1> &paraline, bool b_structure = true)
    {
        pos_world_(0, 0) = paraline(1);
        pos_world_(0, 1) = paraline(4);
        pos_world_(1, 0) = paraline(2);
        pos_world_(1, 1) = paraline(5);
        pos_world_(2, 0) = paraline(3);
        pos_world_(2, 1) = paraline(6);

        if (b_structure)
            vanishing_direction_type_ = paraline(0);
        else
            vanishing_direction_type_ = -1;
        vanishing_direction_ = (pos_world_.block(0, 0, 3, 1) - pos_world_.block(0, 1, 3, 1)).normalized();
        length = (pos_world_.col(0)-pos_world_.col(1)).norm();
    }
    /**
     * @brief reproject the 3D mapline to camera view, we than can obtain whether the mappoint can be detected or not.
     *
     * @param keyframes_Twcs
     * @param add_noise : to generate noisy mappoint
     */
    void AddObservation(std::map<int/*frame_id*/, Mat4/*frame_pose*/> keyframes_Twcs, bool add_noise_to_meas) //TODO: It's better to use frame_id and frame_pose, rather than i;
    {
        for (auto kf_info : keyframes_Twcs)
        {
            int i = kf_info.first; // kf_idx
            // auto Twc = kf_info.second;
            // }
            // for (size_t i = 0; i < keyframes_Twcs.size(); ++i)
            // {
            auto Twc = keyframes_Twcs[i];
            Mat4 Tcw = Twc.inverse();
            Eigen::Matrix3d Rcw = Tcw.block(0, 0, 3, 3);
            Mat32 tcw(Mat32::Zero());
            tcw.block(0, 0, 3, 1) = Tcw.block(0, 3, 3, 1);
            tcw.block(0, 1, 3, 1) = Tcw.block(0, 3, 3, 1);

            Mat32 pos_in_i;
            Vec2 depth_in_i;
            // in the camera coordinate
            pos_in_i = Rcw * pos_world_ + tcw;
            depth_in_i << pos_in_i(2, 0), pos_in_i(2, 1);
            Eigen::Vector3d point_s, point_e;
            point_s = pos_in_i.col(0);
            point_e = pos_in_i.col(1);
            Mat32 uvd_out = Mat32::Zero();
            bool detected = EndpointsOnImage(point_s, point_e,
                                             traject_->cam_intri.width, traject_->cam_intri.height,
                                             traject_->cam_intri.fx, traject_->cam_intri.fy,
                                             traject_->cam_intri.cx, traject_->cam_intri.cy,
                                             uvd_out);
            // TODO:

            // double depth_s = pos_in_i(2, 0);
            // double depth_e = pos_in_i(2, 1);
            // // if (depth_s < 0.01 || depth_e < 0.01)
            // //     continue; // backside of the camera

            // //
            // double u_s = traject_->cam_intri.fx * pos_in_i(0, 0) / pos_in_i(2, 0) + traject_->cam_intri.cx;
            // double v_s = traject_->cam_intri.fy * pos_in_i(1, 0) / pos_in_i(2, 0) + traject_->cam_intri.cy;
            // double u_e = traject_->cam_intri.fx * pos_in_i(0, 1) / pos_in_i(2, 1) + traject_->cam_intri.cx;
            // double v_e = traject_->cam_intri.fy * pos_in_i(1, 1) / pos_in_i(2, 1) + traject_->cam_intri.cy;

            // // pixel on the image plane

            // // Mat32 uvd = Mat32::Zero();
            // // uvd.block(0, 0, 3, 1) << u_s, v_s, depth_s;
            // // uvd.block(0, 1, 3, 1) << u_e, v_e, depth_e;

            // // bool detected =
            // //     LineFeaturesOnImage(uvd, 0, 0, traject_->cam_intri.width, traject_->cam_intri.height, uvd_out);
            // Eigen::Vector3d point_s, point_e;
            // point_s << u_s, v_s, depth_s;
            // point_e << u_e, v_e, depth_e;
            // bool detected = EndpointsOnImage(point_s, point_e,
            //                                  traject_->cam_intri.width, traject_->cam_intri.height, uvd_out);
            // std::cout << "old line:" << u_s << "," << v_s << "," << depth_s << ";" << u_e << "," << v_e << "," << depth_e << std::endl;

            if (!detected)
                continue;
            // std::cout << "old line:" << u_s << "," << v_s << "," << depth_s << ";" << u_e << "," << v_e << "," << depth_e << std::endl;
            // std::cout << "new line:" << uvd_out.col(0) << ";" << uvd_out.col(1) << std::endl;

            auto AddNoise = [&](Vec3 &pixel_d_coord, Vec3 &meas_3d){
                double noise_x, noise_y, noise_z;
                noise_x = pixel_n_(generator_);
                noise_y = pixel_n_(generator_);
                noise_z = depth_n_(generator_); // with noise
                if(noise_z>0.1)
                    noise_z = 0.08;     
                pixel_d_coord(0, 0) += noise_x;
                pixel_d_coord(1, 0) += noise_y;
                pixel_d_coord(2, 0) += noise_z;
                double x_s =
                    pixel_d_coord(2, 0) * (pixel_d_coord(0, 0) - traject_->cam_intri.cx) / traject_->cam_intri.fx;
                double y_s =
                    pixel_d_coord(2, 0) * (pixel_d_coord(1, 0) - traject_->cam_intri.cy) / traject_->cam_intri.fy;
                meas_3d(0, 0) = x_s;
                meas_3d(1, 0) = y_s;
                meas_3d(2, 0) = pixel_d_coord(2, 0);
            };

            Vec3 pixel_d_coord_s, pixel_d_coord_e;
            Vec3 meas_3d_coord_s, meas_3d_coord_e;

            Mat32 pixel_d_coord;

            // AddNoise(pixel_d_coord_s, meas_3d_coord_s);
            // AddNoise(pixel_d_coord_e, meas_3d_coord_e);
            // pixel_d_coord.block(0, 0, 3, 1) = pixel_d_coord_s;
            // pixel_d_coord.block(0, 1, 3, 1) = pixel_d_coord_e;
            // obs_frameid_linepixel_[i] = pixel_d_coord;

            // For 3D lines
            Mat32 meas_3d_in_i; // endpoints representation
            // pixel_d_coord_s << u_s, v_s, depth_s;
            // pixel_d_coord_e << u_e, v_e, depth_e;
            pixel_d_coord_s << uvd_out.col(0)(0), uvd_out.col(0)(1), uvd_out.col(0)(2);
            pixel_d_coord_e << uvd_out.col(1)(0), uvd_out.col(1)(1), uvd_out.col(1)(2);

            if (add_noise_to_meas)
            {
                AddNoise(pixel_d_coord_s, meas_3d_coord_s);
                AddNoise(pixel_d_coord_e, meas_3d_coord_e);
            }

            meas_3d_in_i.block(0, 0, 3, 1) = meas_3d_coord_s;
            meas_3d_in_i.block(0, 1, 3, 1) = meas_3d_coord_e;
            
            pixel_d_coord.block(0, 0, 3, 1) = pixel_d_coord_s;
            pixel_d_coord.block(0, 1, 3, 1) = pixel_d_coord_e;
            obs_frameid_linepixel_[i] = pixel_d_coord;

#ifdef __DEBUG__OFF
                   std::cout <<">>>> in the "<< i <<"th frame, of total " << keyframes_Twcs.size()<<"frames:" << std::endl;
                   std::cout << "groundtruth uv-d:" << u_s <<","<<v_s<<","<<depth_s<<";"
                                                    << u_e <<","<<v_e<<","<<depth_e<<std::endl;
                   std::cout << "noisy uv-d:" << pixel_d_coord_s(0, 0) <<","<<pixel_d_coord_s(1, 0)<<","<<pixel_d_coord_s(2, 0)<<";"
                                              << pixel_d_coord_e(0, 0) <<","<<pixel_d_coord_e(1, 0)<<","<<pixel_d_coord_e(2, 0)<<std::endl;
                   std::cout << "groundtruth endpoints in cam:" <<pos_in_i(0,0) <<","<<pos_in_i(1,0)<<","<<pos_in_i(2,0)<<";"
                                                                <<pos_in_i(0,1) <<","<<pos_in_i(1,1)<<","<<pos_in_i(2,1)<<std::endl;
                   std::cout << "noisy endpoints in cam:" <<meas_3d_in_i(0,0) <<","<<meas_3d_in_i(1,0)<<","<<meas_3d_in_i(2,0)<<";"
                                                          <<meas_3d_in_i(0,1) <<","<<meas_3d_in_i(1,1)<<","<<meas_3d_in_i(2,1)<<std::endl;
#endif

            if (add_noise_to_meas)
                obs_frameid_linepos_[i] = meas_3d_in_i;
            else
                obs_frameid_linepos_[i] = pos_in_i;

            // the distance between gt-line and noisy-line
            Mat32 distance = meas_3d_in_i - pos_in_i;
            double end_distance =  (distance.col(0)-distance.col(1)).norm();
            double length =  (pos_in_i.col(0)-pos_in_i.col(1)).norm();

            observed_++;
        }

        if (observed_<2) //
            std::cout << "The " << this->num_id_ << " mapline didn't be detected by any frames." << std::endl;
    }

    void AddObservation(std::map<int /*frame_id*/, Mat4 /*frame_pose*/> keyframes_Twcs,
                        std::vector<std::pair<int /*point_id*/, Eigen::Matrix<double, 7, 1> /*frame_id*/>> &associate,
                        bool add_noise_to_meas) // TODO: It's better to use frame_id and frame_pose, rather than i;
    {
        for (auto asso_i : associate)
        {
            int line_id = asso_i.first;
            if (num_id_ == line_id)
            {
                int i = asso_i.second[0];
                auto Twc = keyframes_Twcs[i];
                Mat4 Tcw = Twc.inverse();
                Eigen::Matrix3d Rcw = Tcw.block(0, 0, 3, 3);
                Mat32 tcw(Mat32::Zero());
                // Vec3 tcw = Tcw.block(0, 3, 3, 1);

                tcw.block(0, 0, 3, 1) = Tcw.block(0, 3, 3, 1);
                tcw.block(0, 1, 3, 1) = Tcw.block(0, 3, 3, 1);

                Mat32 pos_in_i;
                Vec2 depth_in_i;
                // in the camera coordinate
                pos_in_i = Rcw * pos_world_ + tcw;

                Mat32 uvd_out = Mat32::Zero();
                uvd_out.block(0, 0, 3, 1) << asso_i.second[1], asso_i.second[2], asso_i.second[3];
                uvd_out.block(0, 1, 3, 1) << asso_i.second[4], asso_i.second[5], asso_i.second[6];

                auto AddNoise = [&](Vec3 &pixel_d_coord, Vec3 &meas_3d)
                {
                    double noise_x, noise_y, noise_z;
                    noise_x = pixel_n_(generator_);
                    noise_y = pixel_n_(generator_);
                    noise_z = depth_n_(generator_); // with noise
                    if (noise_z > 0.1)
                        noise_z = 0.08;
                    pixel_d_coord(0, 0) += noise_x;
                    pixel_d_coord(1, 0) += noise_y;
                    // pixel_d_coord(2, 0) += noise_z;
                    std::cout << "z-noise" << pixel_d_coord(2, 0);
                    pixel_d_coord(2, 0) = 35130 / (35130 / (pixel_d_coord(2, 0) + noise_z) + 0.5 + 0.5);
                    std::cout << "," << pixel_d_coord(2, 0) << std::endl;
                    double x_s =
                        pixel_d_coord(2, 0) * (pixel_d_coord(0, 0) - traject_->cam_intri.cx) / traject_->cam_intri.fx;
                    double y_s =
                        pixel_d_coord(2, 0) * (pixel_d_coord(1, 0) - traject_->cam_intri.cy) / traject_->cam_intri.fy;
                    meas_3d(0, 0) = x_s;
                    meas_3d(1, 0) = y_s;
                    meas_3d(2, 0) = pixel_d_coord(2, 0);
                };

                Vec3 pixel_d_coord_s, pixel_d_coord_e;
                Vec3 meas_3d_coord_s, meas_3d_coord_e;

                Mat32 pixel_d_coord;

                // For 3D lines
                Mat32 meas_3d_in_i; // endpoints representation
                // pixel_d_coord_s << u_s, v_s, depth_s;
                // pixel_d_coord_e << u_e, v_e, depth_e;
                pixel_d_coord_s << uvd_out.col(0)(0), uvd_out.col(0)(1), uvd_out.col(0)(2);
                pixel_d_coord_e << uvd_out.col(1)(0), uvd_out.col(1)(1), uvd_out.col(1)(2);

                if (add_noise_to_meas)
                {
                    AddNoise(pixel_d_coord_s, meas_3d_coord_s);
                    AddNoise(pixel_d_coord_e, meas_3d_coord_e);
                }

                meas_3d_in_i.block(0, 0, 3, 1) = meas_3d_coord_s;
                meas_3d_in_i.block(0, 1, 3, 1) = meas_3d_coord_e;

                pixel_d_coord.block(0, 0, 3, 1) = pixel_d_coord_s;
                pixel_d_coord.block(0, 1, 3, 1) = pixel_d_coord_e;
                obs_frameid_linepixel_[i] = pixel_d_coord;

#ifdef __DEBUG__OFF
                std::cout << ">>>> in the " << i << "th frame, of total " << keyframes_Twcs.size() << "frames:" << std::endl;
                std::cout << "groundtruth uv-d:" << u_s << "," << v_s << "," << depth_s << ";"
                          << u_e << "," << v_e << "," << depth_e << std::endl;
                std::cout << "noisy uv-d:" << pixel_d_coord_s(0, 0) << "," << pixel_d_coord_s(1, 0) << "," << pixel_d_coord_s(2, 0) << ";"
                          << pixel_d_coord_e(0, 0) << "," << pixel_d_coord_e(1, 0) << "," << pixel_d_coord_e(2, 0) << std::endl;
                std::cout << "groundtruth endpoints in cam:" << pos_in_i(0, 0) << "," << pos_in_i(1, 0) << "," << pos_in_i(2, 0) << ";"
                          << pos_in_i(0, 1) << "," << pos_in_i(1, 1) << "," << pos_in_i(2, 1) << std::endl;
                std::cout << "noisy endpoints in cam:" << meas_3d_in_i(0, 0) << "," << meas_3d_in_i(1, 0) << "," << meas_3d_in_i(2, 0) << ";"
                          << meas_3d_in_i(0, 1) << "," << meas_3d_in_i(1, 1) << "," << meas_3d_in_i(2, 1) << std::endl;
#endif

                if (add_noise_to_meas)
                    obs_frameid_linepos_[i] = meas_3d_in_i;
                else
                    obs_frameid_linepos_[i] = pos_in_i;

                // the distance between gt-line and noisy-line
                Mat32 distance = meas_3d_in_i - pos_in_i;
                double end_distance = (distance.col(0) - distance.col(1)).norm();
                double length = (pos_in_i.col(0) - pos_in_i.col(1)).norm();

                observed_++;
            }
        }
        if (observed_ < 2) //
            std::cout << "The " << this->num_id_ << " mapline didn't be detected by any frames." << std::endl;
    }

    bool
    LineFeaturesOnImage(Mat32 uvd, double min_x, double min_y, double max_x, double max_y, Mat32 &uvd_out)
    {
        // Completely inside.
        double u_1, v_1, d_1;
        u_1 = uvd(0, 0);
        v_1 = uvd(1, 0);
        d_1 = uvd(2, 0);
        double u_2, v_2, d_2;
        u_2 = uvd(0, 1);
        v_2 = uvd(1, 1);
        d_2 = uvd(2, 1);

        // if inside of the image
        bool inside[2] = {false, false};

#ifdef __DEBUG__
        // std::cout<<"test"<< u_1<<","<<v_1<<","<<u_2<<","<<v_2<<std::endl;
        // cv::Mat img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
        // cv::line(img, cv::Point(u_1, v_1),
        //                      cv::Point(u_2, v_2),
        //                      cv::Scalar(125, 155, 55));
        // cv::circle(img, cv::Point(u_1, v_1), 1, cv::Scalar(125, 155, 55) ) ;                   
        // cv::imshow("test.png", img);
        // cv::waitKey();
#endif

        if ((u_1 >= min_x && u_1 <= max_x) && (v_1 >= min_y && v_1 <= max_y))
        {
            uvd_out.block(0, 0, 3, 1) << u_1, v_1, d_1;
            inside[0] = true; //start_point is valid 
        }
        if ((u_2 >= min_x && u_2 <= max_x) && (v_2 >= min_y && v_2 <= max_y))
        {
            uvd_out.block(0, 1, 3, 1) << u_2, v_2, d_2;
            inside[1] = true;  // end_point is valid
        }
        if (inside[0] && inside[1])  // the 2D line is valid
        {
            uvd_out = uvd;
            return true;
        }
        else
        {
            if (abs(u_2 - u_1) < 0.001)
            {
                if(u_1 > min_x && u_1 < max_x)
                    return true;
                else 
                    return false;    
            }

            //assert((u_2 - u_1)>0.001);
            // line:  y= mx+b
            double m = (v_2 - v_1) / (u_2 - u_1);
            double b = v_2 - m*u_2; 
            
            //std::cout<< "m:"<<m<<", b"<<b<<std::endl;
            std::vector<Vec3> endpoints_cand;
            auto ComputeInlier = [&](double &m, double &b, /*int bound &i --> x=0, x=639, y=0, y=479*/ std::vector<Vec3> &endpoints)
            {
                double y = b;
                //std::cout<<"x,y:"<<0<<","<<y<<std::endl;
                if (y > min_y && y < max_y)
                {
                    // depth info
                    double can_d = 0;
                    //can_d = (0 - u) / (u2 - u) * (d2 - d) + d;
                    Vec3 pixel;
                    pixel << 0, y, can_d;
                    endpoints.push_back(pixel);
                }

                y = m*639+b;
                //std::cout<<"x,y:"<<639<<","<<y<<std::endl;
                if (y > min_y && y < max_y)
                {
                    // depth info
                    double can_d = 0;
                    //can_d = (639 - u) / (u2 - u) * (d2 - d) + d;
                    Vec3 pixel;
                    pixel << 639, y, can_d;
                    endpoints.push_back(pixel);
                }

                double x = -b/m;
                //std::cout<<"x,y:"<<x<<","<<0<<std::endl;

                if (x > min_x && x < max_x)
                {
                    double can_d = 0;
                    //can_d = (x - u) / (u2 - u) * (d2 - d) + d;
                    Vec3 pixel;
                    pixel << x, 0, can_d;
                    endpoints.push_back(pixel);
                }

                x = (479-b)/m;
                //std::cout<<"x,y:"<<x<<","<<479<<std::endl;

                if (x > min_x && x < max_x)
                {
                    double can_d = 0;
                    // can_d = (x - u) / (u2 - u) * (d2 - d) + d;
                    Vec3 pixel;
                    pixel << x, 479, can_d;
                    endpoints.push_back(pixel);
                }

                // for(int i =0; i<endpoints.size(); i++)
                //    std::cout<< endpoints[i]<<std::endl;
            };

            //std::cout<<"finished:"<<std::endl;
            ComputeInlier(m,b, endpoints_cand);
            if(endpoints_cand.size()<2)
                return false;
            // if (endpoints.size() < 2)
            //     return false;

            for (int i = 0; i < endpoints_cand.size(); i++)
            {
                if(i>1)
                    continue;
                if (!inside[i])
                    uvd_out.block(0, i, 3, 1) = endpoints_cand[i];
            }    
            //std::cout<<"finished2:"<<std::endl;

#ifdef __DEBUG__
        // std::cout<<"test2"<< u_1<<","<<v_1<<","<<u_2<<","<<v_2<<std::endl;
        // cv::Mat img2 = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
        // cv::line(img2, cv::Point(u_1, v_1),
        //                      cv::Point(u_2, v_2),
        //                      cv::Scalar(125, 155, 55));
        // cv::circle(img2, cv::Point(u_1, v_1), 1, cv::Scalar(125, 155, 55) ) ;                   
        // cv::imshow("test2.png", img2);
        // cv::waitKey();
#endif
            return true;
        }
    }

    bool EndpointsOnImage(Eigen::Vector3d &pts_s, Eigen::Vector3d &pts_e, double max_x, double max_y,
                          double fx, double fy, double cx, double cy, Mat32 &uvd_out)
    {
        //.normalized();
        double line_distance = (pts_s - pts_e).norm();
        int point_num = 100;
        Eigen::Vector3d sample_direction = (pts_s - pts_e) / 100;
        std::vector<Eigen::Vector3d> sample_pts;

        for (int i = 0; i < 100; i++)
        {
            Eigen::Vector3d pt_i = pts_e + sample_direction * i;
            if (pt_i(2) < 0.01)
                continue;
            double u_i = fx * pt_i(0, 0) / pt_i(2, 0) + cx;
            double v_i = fy * pt_i(1, 0) / pt_i(2, 0) + cy;

            if ((u_i < 0) || (u_i > max_x) || (v_i < 0) || (v_i > max_y))
                continue;

            Eigen::Vector3d uvd;
            uvd << u_i, v_i, pt_i(2);
            sample_pts.push_back(uvd);
        }

        if (sample_pts.size() > 5)
        {
            uvd_out.col(0) << sample_pts[0](0), sample_pts[0](1), sample_pts[0](2);
            uvd_out.col(1) << sample_pts[sample_pts.size() - 1](0), sample_pts[sample_pts.size() - 1](1), sample_pts[sample_pts.size() - 1](2);

            return true;
        }
        return false;
    }

    void print()
    {
        std::cout << "\033[0;31m The global position of mappoint is  \033[0m" << std::endl
                  << pos_world_ << std::endl
                  << "which is detected by " << observed_ << "cameras" << std::endl;
    }
};
} // namespace simulator

#endif // __VENOM_SRC_LANDMARK_ENVLINE_HPP__
