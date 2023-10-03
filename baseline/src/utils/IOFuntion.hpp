/***
 * @Author: ceres lib
 * @Date: 2023-02-09 16:51:29
 * @LastEditTime: 2023-02-09 16:55:18
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /VENOM/src/util/IOFuntion.hpp
 */

#ifndef __VENOM_SRC_UTIL_IOFUNCTION_HPP__
#define __VENOM_SRC_UTIL_IOFUNCTION_HPP__

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include <istream>
#include <string>
#include <vector>

// #include "gtest/gtest.h"
#include "src/utils/UtilStruct.hpp"

namespace simulator
{
  namespace IO
  {

    //
    struct Pose3d
    {
      Eigen::Vector3d p;
      Eigen::Quaterniond q;

      // The name of the data type in the g2o file format.
      static std::string name() { return "VERTEX_SE3:QUAT"; }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline std::istream &operator>>(std::istream &input, Pose3d &pose)
    {
      input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >>
          pose.q.z() >> pose.q.w();
      // Normalize the quaternion to account for precision loss due to
      // serialization.
      pose.q.normalize();
      return input;
    }

    //
    using MapOfPoses =
        std::map<int, Pose3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Pose3d>>>;
    using MapOfMapPoints =
        std::map<int /*mappoint_id*/, Eigen::Vector3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d>>>;
    using MapOfMapLines =
        std::map<int /*mapline_id*/, Eigen::Matrix<double, 3, 2>, std::less<int>,
                 Eigen::aligned_allocator<
                     std::pair<const int, Eigen::Matrix<double, 3, 2>>>>;
    using MapOfMapParaLines =
        std::map<int /*mapparalines_id*/, std::vector<int /*mapline_id*/>, std::less<int>,
                 Eigen::aligned_allocator<std::vector<int>>>;

    using MapOfAssoPoints = std::map<
        int /*mappoint_id*/,
        std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>>,
        std::less<int>,
        Eigen::aligned_allocator<
            std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>>>>;
    using MapOfAssoLines = std::map<
        int /*mapline_id*/,
        std::vector<std::pair<int /*frame_id*/, Eigen::Matrix2d /*2d_meas*/>>,
        std::less<int>,
        Eigen::aligned_allocator<
            std::vector<std::pair<int /*frame_id*/, Eigen::Matrix2d /*2d_meas*/>>>>;

    struct Constraint3d
    {
      int id_begin;
      int id_end;

      // The transformation that represents the pose of the end frame E w.r.t. the
      // begin frame B. In other words, it transforms a vector in the E frame to
      // the B frame.
      Pose3d t_be;

      // The inverse of the covariance matrix for the measurement. The order of the
      // entries are x, y, z, delta orientation.
      Eigen::Matrix<double, 6, 6> information;

      // The name of the data type in the g2o file format.
      static std::string name() { return "EDGE_SE3:QUAT"; }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline std::istream &operator>>(std::istream &input, Constraint3d &constraint)
    {
      Pose3d &t_be = constraint.t_be;
      input >> constraint.id_begin >> constraint.id_end >> t_be;

      for (int i = 0; i < 6 && input.good(); ++i)
      {
        for (int j = i; j < 6 && input.good(); ++j)
        {
          input >> constraint.information(i, j);
          if (i != j)
          {
            constraint.information(j, i) = constraint.information(i, j);
          }
        }
      }
      return input;
    }

    using VectorOfConstraints =
        std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>;

    template <typename Pose, typename Allocator>
    bool ReadVertex(std::ifstream *infile,
                    std::map<int, Pose, std::less<int>, Allocator> *poses)
    {
      int id;
      Pose pose;
      *infile >> id >> pose;

      // Ensure we don't have duplicate poses.
      if (poses->find(id) != poses->end())
      {
        LOG(ERROR) << "Duplicate vertex with ID: " << id;
        return false;
      }
      (*poses)[id] = pose;

      return true;
    }

    // Reads the constraints between two vertices in the pose graph
    template <typename Constraint, typename Allocator>
    void ReadConstraint(std::ifstream *infile,
                        std::vector<Constraint, Allocator> *constraints)
    {
      Constraint constraint;
      *infile >> constraint;

      constraints->push_back(constraint);
    }

    // Reads a file in the g2o filename format that describes a pose graph
    // problem. The g2o format consists of two entries, vertices and constraints.
    //
    // In 2D, a vertex is defined as follows:
    //
    // VERTEX_SE2 ID x_meters y_meters yaw_radians
    //
    // A constraint is defined as follows:
    //
    // EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
    //
    // where I_ij is the (i, j)-th entry of the information matrix for the
    // measurement.
    //
    //
    // In 3D, a vertex is defined as follows:
    //
    // VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
    //
    // where the quaternion is in Hamilton form.
    // A constraint is defined as follows:
    //
    // EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12
    // I_13 ... I_16 I_22 I_23 ... I_26 ... I_66 // NOLINT
    //
    // where I_ij is the (i, j)-th entry of the information matrix for the
    // measurement. Only the upper-triangular part is stored. The measurement order
    // is the delta position followed by the delta orientation.
    template <typename Pose, typename Constraint, typename MapAllocator,
              typename VectorAllocator>
    bool ReadG2oFile(const std::string &filename,
                     std::map<int, Pose, std::less<int>, MapAllocator> *poses,
                     std::vector<Constraint, VectorAllocator> *constraints)
    {
      CHECK(poses != nullptr);
      CHECK(constraints != nullptr);

      poses->clear();
      constraints->clear();

      std::ifstream infile(filename.c_str());
      if (!infile)
      {
        return false;
      }

      std::string data_type;
      while (infile.good())
      {
        // Read whether the type is a node or a constraint.
        infile >> data_type;
        if (data_type == Pose::name())
        {
          if (!ReadVertex(&infile, poses))
          {
            return false;
          }
        }
        else if (data_type == Constraint::name())
        {
          ReadConstraint(&infile, constraints);
        }
        else
        {
          LOG(ERROR) << "Unknown data type: " << data_type;
          return false;
        }

        // Clear any trailing whitespace from the line.
        infile >> std::ws;
      }

      return true;
    }

    
    void ReadPublicTraj(const std::string &traj_path,  std::vector<Eigen::Matrix4d> &vec_traject_gt_Twc) //,  std::vector<Eigen::Matrix4d> &vec_traj)
    {
        std::cout << std::endl 
                << "\033[0;33m[Venom Simulator Printer] Read trajectory from " << traj_path << ".\033[0m" << std::endl;
        std::ifstream file(traj_path);
        std::string line;
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

            vec_traject_gt_Twc.push_back(pose);
        }
        // std::cout<<"vec_traject_gt_Twc:"<<vec_traject_gt_Twc.size()<<std::endl;
        file.close();
    }
 
    /**
     * @brief Read points
     * 
     * @param filename 
     * @param pts 
     */
    void ReadPublicPointClouds(const std::string &filename, std::vector<Eigen::Vector3d> &pts)
    {
      std::cout << std::endl 
                << "\033[0;33m[Venom Simulator Printer] Read Point Landmarks from " << filename << ".\033[0m" << std::endl;
            
      std::ifstream file(filename);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        //std::cout<<words[0]<<std::endl;
        Eigen::Vector3d pt = Eigen::Vector3d::Zero();
        pt.x() = boost::lexical_cast<double>(words[0]);
        pt.y() = boost::lexical_cast<double>(words[1]);
        pt.z() = boost::lexical_cast<double>(words[2]);
        pts.push_back(pt);
      }
      file.close();
    }

    /**
     * @brief Read points
     *
     * @param filename
     * @param pts
     */
    void ReadPublicPointClouds(const std::string &filename, std::vector<std::pair<int, Eigen::Vector3d>> &pts)
    {
      std::ifstream file(filename);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        // std::cout<<words[0]<<std::endl;
        Eigen::Vector3d pt = Eigen::Vector3d::Zero();
        int mp_id = boost::lexical_cast<int>(words[0]);

        pt.x() = boost::lexical_cast<double>(words[1]);
        pt.y() = boost::lexical_cast<double>(words[2]);
        pt.z() = boost::lexical_cast<double>(words[3]);
        // std::cout << "pt.x()" << pt.x() << pt.y() << std::endl;
        pts.push_back(std::make_pair(mp_id, pt));
      }
      file.close();
    }

    /**
     * @brief Read lines
     * 
     * @param filename 
     * @param paralines 
     */
    void ReadPublicLineClouds(const std::string &filename, std::vector<Eigen::Matrix<double,7,1>> &paralines)
    {
      std::cout << std::endl 
                << "\033[0;33m[Venom Simulator Printer] Read Line Landmarks from " << filename << ".\033[0m" << std::endl;

      std::ifstream file(filename);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
                //std::cout<<words[0]<<std::endl;
        Eigen::Matrix<double,7,1> paraline = Eigen::Matrix<double,7,1>::Zero();
        paraline(0) = boost::lexical_cast<double>(words[0]);
        paraline(1) = boost::lexical_cast<double>(words[1]);
        paraline(2) = boost::lexical_cast<double>(words[2]);
        paraline(3) = boost::lexical_cast<double>(words[3]);
        paraline(4) = boost::lexical_cast<double>(words[4]);
        paraline(5) = boost::lexical_cast<double>(words[5]);
        paraline(6) = boost::lexical_cast<double>(words[6]);
        paralines.push_back(paraline);
      }
      file.close();
    }

    void ReadPublicPointAssociation(const std::string &filename,
                                    std::vector<std::pair<int, int>> &asso_ptid_frame_id)
    {
      std::ifstream file(filename);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        std::map<int, int> asso_i;
        if (words[0].compare("MappointFrameAsso:") == 0)
        {
          int point_id = boost::lexical_cast<double>(words[1]);
          int frame_id = boost::lexical_cast<double>(words[2]);

          // std::cout << "boost::lexical_cast<double>(words[1])"
          //           << boost::lexical_cast<double>(words[1]) << "," << point_id << std::endl;
          double u = boost::lexical_cast<double>(words[3]);
          double v = boost::lexical_cast<double>(words[4]);
          // TODO: Yanyan depth

          asso_ptid_frame_id.push_back(std::make_pair(point_id, frame_id));
        }
      }
      file.close();
    }

    void ReadPublicLineAssociation(const std::string &filename,
                                   std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>> &asso_plid_frame_id)
    {
      std::ifstream file(filename);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        // std::cout << words[0] << std::endl;
        std::map<int, int> asso_i;
        if (words[0].compare("MaplineFrameAsso:") == 0)
        {
          Eigen::Matrix<double, 7, 1> observation;
          int line_id = boost::lexical_cast<double>(words[1]);
          double frame_id = boost::lexical_cast<double>(words[2]);
          double u_s = boost::lexical_cast<double>(words[3]);
          double v_s = boost::lexical_cast<double>(words[4]);
          double d_s = boost::lexical_cast<double>(words[5]);
          double u_e = boost::lexical_cast<double>(words[6]);
          double v_e = boost::lexical_cast<double>(words[7]);
          double d_e = boost::lexical_cast<double>(words[8]);
          observation << frame_id, u_s, v_s, d_s, u_e, v_e, d_e;

          // TODO: Yanyan depth

          asso_plid_frame_id.push_back(std::make_pair(line_id, observation));
        }
      }
      file.close();
    }

    void ReadVENOMFactorGraph(const std::string &file_name,
                              simulator::OptimizationManager &paras)
    {

      std::fstream file;
      file.open(file_name.c_str(), std::ios::in);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        //std::cout<<words[0]<<std::endl;
        if (words[0].compare("Vertex:") == 0)
        {
          Eigen::Vector3d trans = Eigen::Vector3d::Zero();
          Eigen::Quaterniond rot;
          int v_index = boost::lexical_cast<int>(words[1]);

          trans.x() = boost::lexical_cast<double>(words[2]);
          trans.y() = boost::lexical_cast<double>(words[3]);
          trans.z() = boost::lexical_cast<double>(words[4]);

          rot.x() = boost::lexical_cast<double>(words[5]);
          rot.y() = boost::lexical_cast<double>(words[6]);
          rot.z() = boost::lexical_cast<double>(words[7]);
          rot.w() = boost::lexical_cast<double>(words[8]);
          Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
          Eigen::Matrix3d R = Eigen::Quaterniond(rot.w(), rot.x(), rot.y(), rot.z()).toRotationMatrix();
          pose.template block<3, 3>(0, 0) = R;
          pose.template block<3, 1>(0, 3) = trans;
          paras.kfs[v_index] = pose;

          // std::cout << "Vertex:" << v_index << std::endl;
        }
        else if (words[0].compare("Mappoint:") == 0)
        {
          int mp_id = boost::lexical_cast<int>(words[1]);
          Eigen::Vector3d pos = Eigen::Vector3d::Zero();

          pos.x() = boost::lexical_cast<double>(words[2]);
          pos.y() = boost::lexical_cast<double>(words[3]);
          pos.z() = boost::lexical_cast<double>(words[4]);
          paras.mappoints[mp_id] = pos;
          // std::cout << "mp_id:" << mp_id << std::endl;
        }
        else if (words[0].compare("Mapline:") == 0)
        {
          int ml_id = boost::lexical_cast<int>(words[1]);
          Eigen::Matrix<double, 3, 2> pos = Eigen::Matrix<double, 3, 2>::Zero();

          pos(0, 0) = boost::lexical_cast<double>(words[2]);
          pos(1, 0) = boost::lexical_cast<double>(words[3]);
          pos(2, 0) = boost::lexical_cast<double>(words[4]);
          pos(0, 1) = boost::lexical_cast<double>(words[5]);
          pos(1, 1) = boost::lexical_cast<double>(words[6]);
          pos(2, 1) = boost::lexical_cast<double>(words[7]);
          paras.maplines[ml_id] = pos;
          // std::cout << "ml_id:" << ml_id << std::endl;
        }
        else if (words[0].compare("MappointFrameAsso:") == 0)
        {
          // std::cout << "e:" << words[0] << std::endl;
          int mp_id = boost::lexical_cast<int>(words[1]);
          int kf_id = boost::lexical_cast<int>(words[2]);

          Eigen::Vector2d feature = Eigen::Vector2d::Zero();

          feature.x() = boost::lexical_cast<double>(words[3]);
          feature.y() = boost::lexical_cast<double>(words[4]);
          paras.asso_mp_meas[mp_id].push_back(std::make_pair(kf_id, feature));
          // std::cout << "mp_id" << mp_id << "," << kf_id << std::endl;
        }
        else if (words[0].compare("MaplineFrameAsso:") == 0)
        {
          int ml_id = boost::lexical_cast<int>(words[1]);
          int kf_id = boost::lexical_cast<int>(words[2]);
          Eigen::Vector4d feature(Eigen::Vector4d::Zero());
          feature(0) = boost::lexical_cast<double>(words[3]);
          feature(1) = boost::lexical_cast<double>(words[4]);

          feature(2) = boost::lexical_cast<double>(words[6]);
          feature(3) = boost::lexical_cast<double>(words[7]);

          Eigen::Matrix<double, 6, 1> feature_d(Eigen::Matrix<double, 6, 1>::Zero());
          feature_d(0) = boost::lexical_cast<double>(words[3]);
          feature_d(1) = boost::lexical_cast<double>(words[4]);
          feature_d(2) = boost::lexical_cast<double>(words[5]);
          feature_d(3) = boost::lexical_cast<double>(words[6]);
          feature_d(4) = boost::lexical_cast<double>(words[7]);
          feature_d(5) = boost::lexical_cast<double>(words[8]);
          paras.asso_ml_meas[ml_id].push_back(std::make_pair(kf_id, feature));

          paras.asso_ml_posi[ml_id].push_back(std::make_pair(kf_id, feature_d));

        }
        else if (words[0].compare("ParalineMaplineAsso:") == 0)
        {
          int vd_id = boost::lexical_cast<int>(words[1]);
          int mls_num = boost::lexical_cast<int>(words[2]);
          // std::cout<<"s"<<mls_num<<std::endl;
          for (int i = 0; i < mls_num; i++)
          {
            int ml_id = boost::lexical_cast<int>(words[3 + i]);
            //std::cout<<"xxs"<<ml_id<<std::endl;

            paras.asso_paralineid_mlids[vd_id].push_back(ml_id);
          }
        }
      }
    }

    bool OutputPoses(const std::string &filename, const MapOfPoses &poses)
    {
      std::fstream outfile;
      outfile.open(filename.c_str(), std::istream::out);
      if (!outfile)
      {
        LOG(ERROR) << "Error opening the file: " << filename;
        return false;
      }
      for (const auto &pair : poses)
      {
        outfile << pair.first << " " << pair.second.p.transpose() << " "
                << pair.second.q.x() << " " << pair.second.q.y() << " "
                << pair.second.q.z() << " " << pair.second.q.w() << '\n';
      }
      return true;
    }

    void SaveFramePredictedTrajectoryLovelyTUM(const std::string &filename, std::map<int /*frame_id*/, Eigen::Matrix4d /*frame_pose*/> &vec_Twcs)
    {
      std::cout << "\033[0;33m[Venom Simulator Printer] Saving keyframe trajectory to " << filename << ".\033[0m" << std::endl;

      std::ofstream pose_file;
      pose_file.open(filename);
      // std::sort(vec_Twcs.begin(), vec_Twcs.end());
      // sort(vec_Twcs.begin(), vec_Twcs.end(), [](const std::map<int, Eigen::Matrix4d> &a, const std::map<int, Eigen::Matrix4d> &b)
      //      { return a.first < b.first; }); // lambda way

      for (auto &Tws_i : vec_Twcs)
      {
        int frame_id = Tws_i.first;
        Eigen::Matrix4d Twc_i = Tws_i.second;
        Eigen::Matrix3d R = Twc_i.block(0, 0, 3, 3);
        Eigen::Quaterniond quat(R);

        Vec3 trans = Twc_i.block(0, 3, 3, 1);
        pose_file << frame_id << " " << trans(0) << " " << trans(1) << " " << trans(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
      }

      pose_file.close();
    }

    void MapPointRecord(const std::string &file_path,
                        std::map<int /*mappoint_id*/, Eigen::Vector3d> &mappoints)
    {
      std::cout << "\033[0;33m[Venom Simulator Printer] Saving "
                   "mappoints to "
                << file_path << ".\033[0m" << std::endl;

      std::ofstream mp_file;
      mp_file.open(file_path);

      for (auto &mappoint : mappoints)
      {
        int mp_id = mappoint.first;
        if (mappoint.second.isZero())
          continue;
        mp_file << "Mappoint:"<< mp_id << " " << mappoint.second.x() << " " << mappoint.second.y()
                << " " << mappoint.second.z() << std::endl;
      }

      mp_file.close();
    }

    void MapLineRecord(const std::string &file_path,
                       std::map<int /*mappoint_id*/, Eigen::Matrix<double, 3, 2>> &maplines)
    {
      std::cout << "\033[0;33m[Venom Simulator Printer] Saving "
                   "maplines to "
                << file_path << ".\033[0m" << std::endl;

      std::ofstream mp_file;
      mp_file.open(file_path);

      for (auto &mapline : maplines)
      {
        int ml_id = mapline.first;
        if (mapline.second.isZero())
          continue;
        mp_file << "Mapline:"<< ml_id << " " << mapline.second(0, 0) << " " << mapline.second(1, 0)
                << " " << mapline.second(2, 0) << " " << mapline.second(0, 1) << " " << mapline.second(1, 1)
                << " " << mapline.second(2, 1)
                << std::endl;
      }
      mp_file.close();
    }

    void CovisibilityGraphRecord(
        const std::string &file_path,
        std::map<
            int /*mappoint_id*/,
            std::vector<std::pair<int /*frame_id*/, Eigen::Vector2d /*2d_meas*/>>>
            &asso_mp_meas)
    {
      std::cout << "\033[0;33m[Venom Simulator Printer] Saving "
                   "mappoints to "
                << file_path << ".\033[0m" << std::endl;

      std::ofstream asso_file;
      asso_file.open(file_path);
      asso_file << "# mp_id"
                << " "
                << " frame_id"
                << " "
                << "2D measurement" << std::endl;

      for (auto &mp_meas : asso_mp_meas)
      {
        int mp_id = mp_meas.first;
        //
        for (auto &meas : mp_meas.second)
        {
          int frame_id = meas.first;
          asso_file << mp_id << " " << frame_id << " " << meas.second.x() << " "
                    << meas.second.y() << std::endl;
        }
      }
      asso_file.close();
    }

    /**
     * @brief
     *
     * @param file_name
     * @param poses
     * @param mappoints
     * @param maplines
     * @param mapparalines
     * @param asso_mappoints
     * @param asso_maplines
     */
    void ReadVENOMFactorGraph(const std::string &file_name, IO::MapOfPoses &poses,
                              IO::MapOfMapPoints &mappoints,
                              IO::MapOfMapLines &maplines,
                              IO::MapOfMapParaLines &mapparalines,
                              IO::MapOfAssoPoints &asso_mappoints,
                              IO::MapOfAssoLines &asso_maplines)
    { // TODO: Yanyan
      std::fstream file;
      file.open(file_name.c_str(), std::ios::in);
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
        if (words[0].compare("VERTEX_SE3:QUAT") == 0)
        {
          Eigen::Vector3d pos = Eigen::Vector3d::Zero();
          Eigen::Quaterniond rot;
          int v_index = boost::lexical_cast<int>(words[1]);

          pos.x() = boost::lexical_cast<double>(words[2]);
          pos.y() = boost::lexical_cast<double>(words[3]);
          pos.z() = boost::lexical_cast<double>(words[4]);

          rot.x() = boost::lexical_cast<double>(words[5]);
          rot.y() = boost::lexical_cast<double>(words[6]);
          rot.z() = boost::lexical_cast<double>(words[7]);
          rot.w() = boost::lexical_cast<double>(words[8]);
        }

        if (words[0].compare("EDGE_SE3:QUAT") == 0)
        {
          // std::cout << "e:" << words[0] << std::endl;
          int a_indx = boost::lexical_cast<int>(words[1]);
          int b_indx = boost::lexical_cast<int>(words[2]);

          Eigen::Vector3d pos = Eigen::Vector3d::Zero();
          Eigen::Quaterniond rot;

          pos.x() = boost::lexical_cast<double>(words[3]);
          pos.y() = boost::lexical_cast<double>(words[4]);
          pos.z() = boost::lexical_cast<double>(words[5]);

          rot.x() = boost::lexical_cast<double>(words[6]);
          rot.y() = boost::lexical_cast<double>(words[7]);
          rot.z() = boost::lexical_cast<double>(words[8]);
          rot.w() = boost::lexical_cast<double>(words[9]);

          // std::cout<<"s:"<<vec_vertex_.size()<<std::endl;
        }

        // only record rotation
        if (words[0].compare("EDGE_SO3:QUAT") == 0)
        {
          // std::cout << "e:" << words[0] << std::endl;
          int a_indx = boost::lexical_cast<int>(words[1]);
          int b_indx = boost::lexical_cast<int>(words[2]);

          Eigen::Quaterniond rot;
          rot.x() = boost::lexical_cast<double>(words[6]);
          rot.y() = boost::lexical_cast<double>(words[7]);
          rot.z() = boost::lexical_cast<double>(words[8]);
          rot.w() = boost::lexical_cast<double>(words[9]);
        }

        if (words[0].compare("MAPPOINT:XYZ") ==
            0) // XYZ mappoint_id pose_id u v x y z
        {
          int mappoint_id = boost::lexical_cast<int>(words[1]);
          int frame_id = boost::lexical_cast<int>(words[2]);

          Eigen::Vector2d feature;
          feature.x() = boost::lexical_cast<double>(words[3]);
          feature.y() = boost::lexical_cast<double>(words[4]);

          Eigen::Vector3d mappoint;
          mappoint.x() = boost::lexical_cast<double>(words[5]);
          mappoint.y() = boost::lexical_cast<double>(words[6]);
          mappoint.z() = boost::lexical_cast<double>(words[7]);
        }

        if (words[0].compare("MAPLINE:XYZ") ==
            0) // XYZ mappoint_id pose_id u v x y z
        {
          int mapline_id = boost::lexical_cast<int>(words[1]);
          int frame_id = boost::lexical_cast<int>(words[2]);

          Eigen::Vector4d feature;
          feature(0) = boost::lexical_cast<double>(words[3]);
          feature(1) = boost::lexical_cast<double>(words[4]);
          feature(2) = boost::lexical_cast<double>(words[5]);
          feature(3) = boost::lexical_cast<double>(words[6]);

          Eigen::Matrix<double, 3, 2> endpoints;
          endpoints(0, 0) = boost::lexical_cast<double>(words[7]);
          endpoints(1, 0) = boost::lexical_cast<double>(words[8]);
          endpoints(2, 0) = boost::lexical_cast<double>(words[9]);
          endpoints(0, 1) = boost::lexical_cast<double>(words[10]);
          endpoints(1, 1) = boost::lexical_cast<double>(words[11]);
          endpoints(2, 1) = boost::lexical_cast<double>(words[12]);
        }
      }
    }

    /**
     * @brief 
     * 
     * @param file_name 
     */
    void SaveSystemConfig(const std::string &file_name, simulator::EnvironmentParameter &env_para )
    {
        // record those parameters
        std::ofstream file(file_name);
        file << "The settings of the Venom Simulator system are saved in this file: " << std::endl
              << "frame_num_:" << env_para.frame_num_ << std::endl
              << "traject_type_:" << env_para.traject_type_ << std::endl
              << "vert_lines_:" << env_para.vert_lines_ << std::endl
              << "horiz_lines_:" << env_para.horiz_lines_ << std::endl
              << "vert_points_:" << env_para.vert_points_ << std::endl
              << "horiz_points_:" << env_para.horiz_points_ << std::endl;
        // TODO: intransic matrix
        file.close();
    }

    void SaveVENOMFactorGraph(const std::string &file_name, IO::MapOfPoses &poses,
                              IO::MapOfMapPoints &mappoints,
                              IO::MapOfMapLines &maplines,
                              IO::MapOfMapParaLines &mapparalines,
                              IO::MapOfAssoPoints &asso_mappoints,
                              IO::MapOfAssoLines &asso_maplines)
    {
      //
      std::cout << "\033[0;33m[Venom Simulator Printer] Saving "
                   "maplines to "
                << file_name << ".\033[0m" << std::endl;

      std::ofstream venom_file;
      venom_file.open(file_name);

      // --->
      for (auto &pose : poses)
      {
        int frame_id = pose.first;
        venom_file << "VERTEX_SE3:QUAT"
                   << " " << frame_id << " " << pose.second.p.x()
                   << " " << pose.second.p.y() << " " << pose.second.p.z() << " "
                   << pose.second.q.x() << " " << pose.second.q.y() << " "
                   << pose.second.q.z() << " " << pose.second.q.w() << std::endl;
      }
      for (auto &mappoint : mappoints)
      {
        int mp_id = mappoint.first;
        if (mappoint.second.isZero())
          continue;
        venom_file << "MAPPOINT:XYZ"
                   << " " << mp_id << " " << mappoint.second(0) << " "
                   << mappoint.second(1) << " " << mappoint.second(2) << std::endl;
      }
      for (auto &mapline : maplines)
      {
        int ml_id = mapline.first;
        if (mapline.second.isZero())
          continue;
        venom_file << "MAPLINE:XYZ"
                   << " " << ml_id << " " << mapline.second(0, 0) << " "
                   << mapline.second(1, 0) << " " << mapline.second(2, 0) << " "
                   << mapline.second(0, 1) << " " << mapline.second(1, 1) << " "
                   << mapline.second(2, 1) << std::endl;
      }
      for (auto &mapparaline : mapparalines)
      {
        int mpl_id = mapparaline.first;
        if (mapparaline.second.size() == 0)
          continue;
        venom_file << "MAPPARALINE:XYZ "
                   << " " << mpl_id;

        for (int i = 0; i < mapparaline.second.size(); i++)
        {
          venom_file << " " << mapparaline.second.at(i);
          if (i + 1 == mapparaline.second.size())
            venom_file << std::endl;
        }
      }

      for (auto &asso_mappoint : asso_mappoints)
      {
        int mappoint_id = asso_mappoint.first;
        if (asso_mappoint.second.size() == 0)
          continue;
        venom_file << " ASSO_MAPPOINT "
                   << " " << mappoint_id;
        for (int i = 0; i < asso_mappoint.second.size(); i++)
        {
          int frame_id = asso_mappoint.second.at(i).first;
          Eigen::Vector2d feature = asso_mappoint.second.at(i).second;
          venom_file << " " << frame_id << " " << feature.x() << " " << feature.y() << std::endl;
        }
      }

      for (auto &asso_mapline : asso_maplines)
      {
        int mapline_id = asso_mapline.first;
        if (asso_mapline.second.size() == 0)
          continue;
        venom_file << " ASSO_MAPPOINT "
                   << " " << mapline_id;
        for (int i = 0; i < asso_mapline.second.size(); i++)
        {
          int frame_id = asso_mapline.second.at(i).first;
          Eigen::Matrix2d feature = asso_mapline.second.at(i).second;
          venom_file << " " << frame_id << " " << feature(0, 0) << " " << feature(1, 0) << " " << feature(0, 1) << " " << feature(1, 1) << std::endl;
        }
      }

      venom_file.close();
    }

    void RecordPrivateData(const std::string &folder_path,  simulator::OptimizationManager &opti_para)
        {
#ifdef __VERBOSE__
            std::cout << "The path of the record sequence is: "
                      << folder_path << std::endl;
#endif
            std::ofstream file(folder_path + "venom_sequence.txt");
            std::map<int /*kf_id*/, Eigen::Matrix4d> kfs;
            for (auto &kf : opti_para.kfs)
            {
                int kf_id = kf.first;
                Eigen::Matrix4d Twc_i = kf.second;
                Eigen::Matrix3d R = Twc_i.block(0, 0, 3, 3);
                Eigen::Quaterniond quat(R);

                Vec3 trans = Twc_i.block(0, 3, 3, 1);
                file << "Vertex: " << kf_id << " " << trans(0) << " "
                     << trans(1) << " " << trans(2) << " " << quat.x() << " "
                     << quat.y() << " " << quat.z() << " " << quat.w()
                     << std::endl;
                // std::cout << "kf_id:" << kf_id << std::endl;
            }
            for(auto &mappoint : opti_para.mappoints)
            {
                int mp_id = mappoint.first;
                file << "Mappoint: " << mp_id << " " << mappoint.second.x() << " " << mappoint.second.y() << " " << mappoint.second.z() << std::endl;
                // std::cout << "mp_id:" << mp_id << std::endl;
            }
            for (auto &mapline : opti_para.maplines) {
                int ml_id = mapline.first;
                file << "Mapline: " << ml_id << " " << mapline.second(0, 0)
                     << " " << mapline.second(1, 0) << " "
                     << mapline.second(2, 0) << " " << mapline.second(0, 1)
                     << " " << mapline.second(1, 1) << " "
                     << mapline.second(2, 1) << std::endl;
            }
            for (auto &asso_mp_mea : opti_para.asso_mp_meas) {
                int mp_id = asso_mp_mea.first;
                for (int i = 0; i < asso_mp_mea.second.size(); i++) {
                    int frame_id = asso_mp_mea.second.at(i).first;
                    file << "MappointFrameAsso: " << mp_id << " " << frame_id
                         << " " << asso_mp_mea.second.at(i).second(0, 0) << " "
                         << asso_mp_mea.second.at(i).second(1, 0) << " "
                         << std::endl;
                }
            }
            for (auto &asso_ml_mea : opti_para.asso_ml_posi) {
                int ml_id = asso_ml_mea.first;
                for (int i = 0; i < asso_ml_mea.second.size(); i++) {
                    int frame_id = asso_ml_mea.second.at(i).first;
                    file << "MaplineFrameAsso: " << ml_id << " " << frame_id
                         << " " << asso_ml_mea.second.at(i).second(0, 0) << " "
                         << asso_ml_mea.second.at(i).second(1, 0) << " "
                         << asso_ml_mea.second.at(i).second(2, 0) << " "
                         << asso_ml_mea.second.at(i).second(3, 0) << " "
                         << asso_ml_mea.second.at(i).second(4, 0) << " "
                         << asso_ml_mea.second.at(i).second(5, 0) << " "
                         << std::endl;
                }
            }
            for(auto &asso_paralineid_mlid : opti_para.asso_paralineid_mlids)
            {
                int vd_id = asso_paralineid_mlid.first;
                int mls_size = asso_paralineid_mlid.second.size();
                file<< "ParalineMaplineAsso: " << vd_id <<" " << mls_size<< " ";
                for(int i = 0; i<asso_paralineid_mlid.second.size(); i++)
                    file<< asso_paralineid_mlid.second.at(i) << " ";
                file<<std::endl;
            }
            file.close();  
        }
  } // namespace IO

} // namespace simulator
#endif //__VENOM_SRC_UTIL_IOFUNCTION_HPP__
