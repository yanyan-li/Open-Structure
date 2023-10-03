/***
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2022-10-06 02:37:57
 * @LastEditTime: 2023-02-09 17:14:15
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /VENOM/src/visulizer/Interface.hpp
 */
#ifndef __VENOM_SRC_VISUALIZER_INTERFACE_HPP__
#define __VENOM_SRC_VISUALIZER_INTERFACE_HPP__
#include <eigen3/Eigen/Dense>
#include <experimental/filesystem>
#include <opencv2/core/persistence.hpp>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "src/estimator/Track.hpp"
#include "src/manager_env/EnvManager.hpp"
#include "src/manager_env/EnvTrajectory.hpp"
#include "src/map/MapLine.hpp"
#include "src/map/MapPoint.hpp"

#include "src/optimizer/CovisExtensPointParalineBA.hpp"
#include "src/optimizer/CovisPointBA.hpp"
#include "src/optimizer/CovisPointLineBA.hpp"
#include "src/optimizer/CovisPointParalineBA.hpp"
#include "src/optimizer/PoseGraphOptimization.hpp"

#include "src/utils/IOFuntion.hpp"
#include "src/utils/UtilStruct.hpp"
namespace simulator
{
    class Interface
    {

    public:
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        simulator::EnvManager *ptr_env_manager_;
        IO::MapOfPoses poses_;              
        IO::VectorOfConstraints constraints_;
        // trajectory of the robot
        std::vector<Mat4> Twcs_true_;
        std::vector<Mat4> Twcs_;
        // edges of two vectices
        std::vector<Vec3/*dge_type*/ /*vertex_id*/ /*vertex_id*/> vertex_connections_;  
        // 3D landmarks of the environment
        std::vector<Vec3> points_gt_;
        std::vector<Mat32> lines_gt_;
        simulator::Trajectory *ptr_robot_trajectory_;
        simulator::Track *ptr_tracker_;
        //
        std::vector<std::vector<std::pair<int, Vec3>>> point_obs_;
        std::vector<double> tri_point_inverse_depth_;
        std::vector<Vec3> tri_point_xyz_;

        std::map<int, Eigen::Vector3d> optimized_mappoints_;
        std::map<int, Mat32> optimized_maplines_;
        //
        std::vector<std::vector<std::pair<int /*frame_id*/, Eigen::Matrix3d /*Rcm*/>>> venom_association_;
        std::vector<int> vec_anchor_id_;
        // rotation estimation
        std::vector<std::vector<std::pair<int, Eigen::Matrix3d>>> rotation_from_venom_;
        std::vector<std::vector<std::pair<int, Eigen::Matrix3d>>> rotation_from_groundtruth_;
        // predicted rot_cw
        std::vector<std::pair<int /*frame_id*/, Mat4 /*frame_pose*/>> vec_traject_estimated_Twc_;

    private:
        //--> parameters
        simulator::EnvironmentParameter env_para_;
        simulator::OptimizationManager opti_para_;
        int frame_num_ = 100;
        int traject_type_;
        int vert_lines_;
        int horiz_lines_;
        int vert_points_;
        int horiz_points_;
        std::string root_path_;
        std::vector<Vec3> points_gt;
        std::vector<Mat32> lines_gt;

        Eigen::Vector3d color_initial_;
        std::vector<Vec3> points_initial_;

        Eigen::Vector3d color_pl_initial_;
        std::vector<Mat32> lines_intial_;

        pangolin::OpenGlRenderState s_cam;
        pangolin::View d_cam;
        int UI_WIDTH;

    public:
        Interface(const std::string &version)
        {
            std::string window_title = "VENOM SLAM SIMULATOR. @" + version;
            pangolin::CreateWindowAndBind(window_title, 1024, 768);
            // 3D Mouse handler requires depth testing to be enabled
            glEnable(GL_DEPTH_TEST);
            // Issue specific OpenGl we might need
            // glEnable(GL_BLEND);
            // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            s_cam = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.7, 10.8, 0, 0, -50.0, 0.0, 10.0, 10.0));
            // Choose a sensible left UI Panel width based on the width of 30
            // charectors from the default font.
            UI_WIDTH = 35 * pangolin::default_font().MaxWidth();
            // Add named OpenGL viewport to window and provide 3D Handler
            d_cam = pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(s_cam));
        }
        
        void StartVenom(const std::string &str_settings)
        {
            // parameters setting
            cv::FileStorage venom_settings(str_settings.c_str(), cv::FileStorage::READ);
            if (!venom_settings.isOpened())
                std::cerr << "\033[0;35m[Venom Similator Printer]Failed to open settings file at:\033[0m"
                          << str_settings << std::endl;
            // home page
            PageHomeSurface(venom_settings);
        }

        void PageHomeSurface(cv::FileStorage &venom_settings)
        {
            venom_settings["Evaluation.result_path"] >> root_path_;
            if (mkdir(root_path_.c_str(), 0777) == -1)
                std::cerr << "Error :  " << strerror(errno) << std::endl;
            else
                std::cout << "Directory created";

            pangolin::CreatePanel("homemenu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
            // background color
            pangolin::Var<int> menuWhite("homemenu.BG-Color Perferences", 10, 0, 10);
            // inputs
            pangolin::Var<bool> menuUsePrivateEnv("homemenu.Open-Structure SLAM Pipeline", false, true);
            pangolin::Var<bool> menuUsePublicFactorGraph("homemenu.Module Evaluation in Param_Opti", false, true);

            int wait_click = 0;
            while (!pangolin::ShouldQuit() && !wait_click)
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);
                //-------> background
                if (!menuWhite)
                    glClearColor(.0f, .0f, .0f, .0f);
                else
                    glClearColor(0.1 * menuWhite, 0.1 * menuWhite, 0.1 * menuWhite, 0.1 * menuWhite);

                if (menuUsePublicFactorGraph)
                    wait_click = 1;
                else if (menuUsePrivateEnv)
                    wait_click = 2;
                pangolin::FinishFrame();
            }
            if (wait_click == 1)
                StartPublicPipeline(venom_settings);  // page for dealing with venom_sequence.txt
            else if (wait_click == 2)
                PageStartPrivatePipeline(venom_settings); // page for dealing with new environments
        }

        // Page------------------> Public Dataset
        void StartPublicPipeline(cv::FileStorage &venom_settings)
        {
            pangolin::CreatePanel("publicmenu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
            // public dataset: read g2o_file
            pangolin::Var<std::string> menuInputPoseGraphFilePath("publicmenu.Path of g2o Pose Graph:", "");
            pangolin::Var<bool> menuStartInputPoseGraph("publicmenu.Input the Pose Graph", false, false);

            pangolin::Var<std::string> menuInputFactorGraphFilePath("publicmenu.Path of VENOM Factor Graph:", "");
            pangolin::Var<bool> menuStartInputFactorGraph("publicmenu.Input the Factor Graph", false, false);  

            pangolin::Var<int> set_vertices_num("publicmenu.Num of Poses:", 0);
            pangolin::Var<int> set_edges_num("publicmenu.Num of Pose Constraints:", 0);
            
            pangolin::Var<int> set_mappoints_num("publicmenu.Num of MapPoints:", 0);
            pangolin::Var<int> set_maplines_num("publicmenu.Num of MapLines:", 0);
            pangolin::Var<int> set_mapparalines_num("publicmenu.Num of MapParaLines:", 0);

            pangolin::Var<bool> prepare_optimization("publicmenu.Optimization Preparation ", false, false); 
            // pose graph optimization
            pangolin::Var<bool> menuPoseGraphOptimization("publicmenu.Pose Graph Optimization", false, false);     
            // our optimization
            // pangolin::Var<bool> menuPointBAGTSAM("publicmenu.CovisPointBA GTSAM", false, false);
            pangolin::Var<bool> menuPointBACeres("publicmenu.CovisPointBA", false, false);
            pangolin::Var<bool> menuPointLineBA("publicmenu.CovisPointLineBA", false, false);
            pangolin::Var<bool> menuPointLineBACE("publicmenu.CovisExtensImpliPointLineBA", false, false);
            pangolin::Var<bool> menuPointLineBACEEx("publicmenu.CovisExtensExpliPointLineBA", false, false);

            pangolin::Var<bool> menuShowTrajectory("publicmenu.Show Trajectory", false, true);
            pangolin::Var<bool> menuShowPoseGraphTrajectory("publicmenu.Show Optimized Trajectory", false, true);
            pangolin::Var<bool> menuShowConnections("publicmenu.Show Connections", false, true);

            // 
            bool click_input_pose_graph = true;
            bool click_reinput = true;
            bool click_input_factor_graph = true;

            bool click_input_data = true;
            bool click_start_optimize = true;

            // bool click_show_connection = true;

            std::map<int, Mat4> Twcs_factorgraph;
            std::vector<Mat4> Twcs_posegraph;
            simulator::OptimizationManager venom_factorgraph;

            IO::MapOfPoses poses;
            IO::MapOfMapPoints map_points;
            IO::MapOfMapLines map_lines;
            IO::MapOfMapParaLines map_paralines;
            IO::MapOfAssoPoints asso_mappoints;
            IO::MapOfAssoLines asso_maplines;

            while(!pangolin::ShouldQuit())
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);

                if(menuStartInputPoseGraph  && click_input_pose_graph)
                {
                    // check menuInputPoseGraphFilePath 
                    menuInputFactorGraphFilePath.Detach();   
                    menuStartInputFactorGraph.Detach(); 
                    //menuStartFactorGraphOptimization.Detach(); 
                    // 1. existed check
                    std::ifstream infile(menuInputPoseGraphFilePath.Get().c_str());
                    if (!infile) {
                        std::cout<<" Load g20 file failed. Please Check your path."<<std::endl;
                        menuInputPoseGraphFilePath=" "; // clean the input path
                        menuStartInputPoseGraph = false;
                    }
                    else
                    {
                        std::pair<int, int> graph_size;
                        DealwithPoseGraph(menuInputPoseGraphFilePath.Get().c_str(), graph_size);
                        if (poses_.size() == 0 || constraints_.size() == 0)
                        {
                            std::cout << "There are no vertices or constraints in the file." << std::endl;
                            menuInputPoseGraphFilePath=" ";
                            menuStartInputPoseGraph = false;
                        }
                        else
                        { 
                            // loal sucessfully
                            set_vertices_num = graph_size.first;
                            set_edges_num = graph_size.second;
                            // set vertices and edges
                            SetPoseGraph(); 
                            click_input_pose_graph = false;
                            click_input_data = false;
                            menuShowTrajectory = true;
                        }
                    }
                }
                else if(menuStartInputFactorGraph && click_input_factor_graph )
                {
                    //
                    menuInputPoseGraphFilePath.Detach();
                    menuStartInputPoseGraph.Detach();
                    prepare_optimization.Detach();
                    menuPoseGraphOptimization.Detach();
                    // menuStartPoseGraphOptimization.Detach();
                    // check menuStartInputFactorGraph

                    Eigen::Vector4d graph_size;

                    // 1. existed check
                    if (!DealWithVenomDataset(menuInputFactorGraphFilePath.Get().c_str(), venom_factorgraph))
                    {
                        std::cout << " Load VENOM file failed. Please Check your path." << std::endl;
                        menuInputFactorGraphFilePath = " "; // clean the input path
                        menuStartInputFactorGraph = false;
                    }
                    else
                    {
                        graph_size(0) = venom_factorgraph.kfs.size();
                        graph_size(1) = venom_factorgraph.mappoints.size();
                        graph_size(2) = venom_factorgraph.maplines.size();
                        graph_size(3) = venom_factorgraph.asso_paralineid_mlids.size();

                        set_vertices_num = graph_size(0);
                        set_mappoints_num = graph_size(1);
                        set_maplines_num = graph_size(2);
                        set_mapparalines_num = graph_size(3);

                        // SET
                        // SetCameraInfo(venom_settings);
                        ptr_robot_trajectory_ = new simulator::Trajectory(venom_settings);

                        set_edges_num.Detach();
                        menuStartInputFactorGraph.Detach();

                        click_input_factor_graph = false;
                        click_input_data = false;
                        menuShowTrajectory = true;
                    }
                }

                if (!click_input_pose_graph && click_start_optimize) {
                    if (menuPoseGraphOptimization) {
                         Twcs_posegraph.clear();
                        optimizer::PoseGraphOptimization::optimizer(poses_, constraints_, Twcs_posegraph);
                        click_start_optimize = false;
                    }
                }

                if (!click_input_factor_graph && click_start_optimize)
                {

                    if (menuPointBACeres)
                    {
                        simulator::optimizer::CovisPointBA::optimizer(ptr_robot_trajectory_,
                                                                                         venom_factorgraph,
                                                                                         optimized_mappoints_,
                                                                                         Twcs_factorgraph);
                        menuShowPoseGraphTrajectory = true;
                        //menuShowOptimizedMapPoints = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;

                        click_start_optimize = false;
                        // menuPointBAGTSAM.Detach();
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                        menuPointLineBACE.Detach();
                        menuPointLineBACEEx.Detach();
                        IO::SaveFramePredictedTrajectoryLovelyTUM(root_path_+"Point_BA.txt", Twcs_factorgraph);
                        IO::MapPointRecord(root_path_+"Point_Mappoint.txt", optimized_mappoints_);
                    }
                    else if (menuPointLineBA)
                    {

                        simulator::optimizer::CovisPointLineBA::optimizer(ptr_robot_trajectory_,
                                                                          venom_factorgraph,
                                                                          optimized_mappoints_,
                                                                          optimized_maplines_,
                                                                          Twcs_factorgraph);
                        menuShowPoseGraphTrajectory = true;
                        // menuShowOptimizedMapPoints = true;
                        // menuShowOptimizedMapLines = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;
                        click_start_optimize = false;
                        // menuPointBAGTSAM.Detach();
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                        menuPointLineBACE.Detach();
                        menuPointLineBACEEx.Detach();
                        IO::SaveFramePredictedTrajectoryLovelyTUM(root_path_+"PointLine_BA.txt", Twcs_factorgraph);
                        IO::MapPointRecord(root_path_+"PointLine_mappoint.txt", optimized_mappoints_);
                        IO::MapLineRecord(root_path_+"PointLine_mapline.txt", optimized_maplines_);
                    }
                    else if (menuPointLineBACE)
                    {

                        simulator::optimizer::CovisPointParalineBA::optimizer(ptr_robot_trajectory_,
                                                                                        venom_factorgraph,
                                                                                        optimized_mappoints_,
                                                                                        optimized_maplines_,
                                                                                        Twcs_factorgraph);
                        menuShowPoseGraphTrajectory = true;
                        // menuShowOptimizedMapPoints = true;
                        // menuShowOptimizedMapLines = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;
                        click_start_optimize = false;
                        // menuPointBAGTSAM.Detach();
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                        menuPointLineBACE.Detach();
                        menuPointLineBACEEx.Detach();
                        IO::SaveFramePredictedTrajectoryLovelyTUM(root_path_+"PointParaline_BA.txt", Twcs_factorgraph);
                        IO::MapPointRecord(root_path_+"PointParaline_Mappoint.txt", optimized_mappoints_);
                        IO::MapLineRecord(root_path_+"PointParaline_Mapline.txt", optimized_maplines_);
                    }
                    else if (menuPointLineBACEEx)
                    {

                        simulator::optimizer::CovisExtensPointParalineBA::optimizer(ptr_robot_trajectory_,
                                                                                        venom_factorgraph,
                                                                                        optimized_mappoints_,
                                                                                        optimized_maplines_,
                                                                                        Twcs_factorgraph);
                        menuShowPoseGraphTrajectory = true;
                        // menuShowOptimizedMapPoints = true;
                        // menuShowOptimizedMapLines = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;
                        click_start_optimize = false;
                        // menuPointBAGTSAM.Detach();
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                        menuPointLineBACE.Detach();
                        menuPointLineBACEEx.Detach();
                        IO::SaveFramePredictedTrajectoryLovelyTUM(root_path_+"PointParaline_JFG_BA.txt", Twcs_factorgraph);
                        IO::MapPointRecord(root_path_+"PointParaline_JFG_Mappoint.txt", optimized_mappoints_);
                        IO::MapLineRecord(root_path_+"PointParaline_JFG_Mapline.txt", optimized_maplines_);

                    }
                }

                if( menuShowPoseGraphTrajectory)
                {
                    
                    std::vector<pangolin::OpenGlMatrix> MsTrue;
                    DrawTrajectoryConnection(MsTrue, Twcs_posegraph, Vec3(0,1,1));
                    // DrawTrajectoryCamera(MsTrue,Vec3(0,0,1));
                }

                if (menuShowTrajectory)
                {
                    std::vector<pangolin::OpenGlMatrix> MsTrue;
                    if (!click_input_factor_graph)
                    {
                        for (auto &mit : venom_factorgraph.kfs)
                            Twcs_true_.push_back(mit.second);
                    }

                    DrawTrajectoryConnection(MsTrue, Twcs_true_, Vec3(0.8, 0.1, 0.1));
                    // DrawTrajectoryCamera(MsTrue, Vec3(0,0,1));
                }

                if (menuShowConnections)
                {
                    //TODO: 这个要和 优化前 还是 后的pose 绑定
                    DrawTrajectoryConnection(vertex_connections_);
                    // click_show_connection = false;
                }
                pangolin::FinishFrame();    
            }
        }

        void DealwithPoseGraph(const std::string &path_g2ofile, std::pair<int, int> &graph_size)
        {
            std::pair<int, int> pose_graph_size;
            
            std::cout << "path:" << path_g2ofile << std::endl;
            std::cout << std::endl
                      << "\033[0;35m[Venom Similator Printer]The path of the "
                         "input file:"
                      << path_g2ofile << "\033[0m" << std::endl;

            // tested
            IO::ReadG2oFile(path_g2ofile, &poses_, &constraints_);
            graph_size.first = poses_.size();
            graph_size.second = constraints_.size();
        }

        void SetPoseGraph()
        {
            // from camera poses to Eigen::
            Twcs_true_.clear();
            vertex_connections_.clear();
            for(auto &pose : poses_)
            {
                Mat4 Twc_true = Mat4::Identity();
                // int pose_id = pose.first;
                // translation
                Twc_true.template block<3,1>(0,3) = pose.second.p;
                // rotation
                // pose.second.q;
                Twcs_true_.push_back(Twc_true);
            }

            for(auto &constraint: constraints_)
            {
                int type = -1;  
                if(constraint.name()=="EDGE_SE3:QUAT")
                    type = 0;
                else if(constraint.name()=="EDGE_SO3:QUAT") 
                    type = 1;    

               vertex_connections_.push_back(Vec3(type, constraint.id_begin, constraint.id_end));
            }
        }

        bool DealWithVenomDataset(const std::string &path_venomfile,
                                  simulator::OptimizationManager &factorgraph)
        {
            std::cout << "path:" << path_venomfile << std::endl;
            std::cout << std::endl
                      << "\033[0;35m[Venom Similator Printer]The path of the "
                         "input file:"
                      << path_venomfile << "\033[0m" << std::endl;

            
            // tested
            IO::ReadVENOMFactorGraph(path_venomfile, factorgraph);
            // poses, map_points,  map_lines, map_paralines, asso_mappoints, asso_maplines);
            std::cout<<"read a venom sequence..."<<std::endl;


            return true;
        }

        // Page-----------------> Private Pipeline
        void PageStartPrivatePipeline(cv::FileStorage &venom_settings)
        {
            pangolin::CreatePanel("privatemenu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
            pangolin::Var<int> traject_num("privatemenu.Frame Number", 100, 50, 4000);
            // pangolin::Var<std::string> intro_traject_type("privatemenu.0->Circle;1->Sphere;2->Corridor.", " ");
            pangolin::Var<int> traject_type("privatemenu.Traj: 0:Circle;1:Sphere;2:Corrid;", 0, 0, 2);
            pangolin::Var<int> vert_points("privatemenu.Point Landmarks", 260, 4, 5000);
            // pangolin::Var<int> horiz_points("privatemenu.Horizontal Points", 10, 0, 20);
            pangolin::Var<int> vert_lines("privatemenu.Line Landmarks", 20, 4, 50);
            // pangolin::Var<int> horiz_lines("privatemenu.Horizontal Lines", 10, 0, 50);
            pangolin::Var<bool> set_env("privatemenu.Decide Settings", false, false);

            // Show user settings
            pangolin::Var<std::string> set_traject_type("privatemenu.Selected Trajectory Type:", " ");
            pangolin::Var<int> set_traject_num("privatemenu.Num of KeyFrames:", 0);

            // pangolin::Var<int> set_vert_points("privatemenu.Num of Vertical Points:", 0);
            // pangolin::Var<int> set_horiz_points("privatemenu.Num of Horizontal Points:", 0);
            // pangolin::Var<int> set_vert_lines("privatemenu.Num of Vertical Lines:", 0);
            // pangolin::Var<int> set_horiz_lines("privatemenu.Num of Horizontal Lines:", 0);

            pangolin::Var<int> set_vert_points("privatemenu.Num of Point Landmarks:", 0);
            // pangolin::Var<int> set_horiz_points("privatemenu.Num of Horizontal Points:", 0);
            pangolin::Var<int> set_vert_lines("privatemenu.Num of Line Landmarks:", 0);
            // pangolin::Var<int> set_horiz_lines("privatemenu.Num of Horizontal Lines:", 0);

            int set_horiz_points;
            int set_horiz_lines;

            // Build System
            pangolin::Var<bool> start_traject_env("privatemenu.Show Cameras & Environment", false, false);
            // show
            pangolin::Var<bool> menuShowTrajectory("privatemenu.Groundruth Trajectory", false, true);
            pangolin::Var<bool> menuShowPoint("privatemenu.Groundtruth Point", false, true);
            pangolin::Var<bool> menuShowLine("privatemenu.Groundtruth Line", false, true);

            // record it as a sequence
            // pangolin::Var<bool> menuSave("privatemenu.Record This Sequence", false, false);
            pangolin::Var<bool> menuPrepareTracking("privatemenu.Start Front-end", false, false);

            bool click_build_env = true;
            bool click_set_start_means = true;
            // bool click_save_dataset = true;
            bool click_start_track = true;
            bool click_read_public = false;
            venom_settings["Env.Public.bool"] >> click_read_public;
            
            while (!pangolin::ShouldQuit())
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);
                if (click_read_public)
                {
                    traject_num.Detach();
                    traject_type.Detach();
                    vert_points.Detach();
                    // horiz_points.Detach();
                    vert_lines.Detach();
                    // horiz_lines.Detach();
                    traject_type = 9;
                }
                
                if (set_env && click_build_env)
                {
                    click_build_env = false;
                    // show env parameters
                    set_traject_num = traject_num;
                    if (traject_type == 0)
                        set_traject_type = "Cycle";
                    else if (traject_type == 1)
                        set_traject_type = "Sphere";
                    else if (traject_type == 2)
                        set_traject_type = "Corridor";
                    else if (traject_type == 9)
                        set_traject_type = "OpenStructure Dataset";

                    // get size parameters of the env
                    if(click_read_public)  // set parameters of public datasets
                    {
                        std::string envpoint_path;
                        std::vector<Eigen::Vector3d> pts_t;
                        venom_settings["Env.Public.envpoint_path"] >> envpoint_path;
                        IO::ReadPublicPointClouds(envpoint_path, pts_t);
                        set_vert_points = pts_t.size();
                        set_horiz_points = 0;

                        std::string envline_path;
                        std::vector<Eigen::Matrix<double, 7, 1>> paralines_t;
                        venom_settings["Env.Public.envline_path"] >> envline_path;
                        IO::ReadPublicLineClouds(envline_path, paralines_t);
                        set_vert_lines = paralines_t.size();
                        set_horiz_lines = 0;

                        std::string traj_path;
                        std::vector<Eigen::Matrix4d> vec_traject_gt_Twc_t;
                        venom_settings["Env.Public.envtraj_path"] >> traj_path;
                        IO::ReadPublicTraj(traj_path, vec_traject_gt_Twc_t);
                        set_traject_num = vec_traject_gt_Twc_t.size();

                        pts_t.clear();
                        paralines_t.clear();
                        vec_traject_gt_Twc_t.clear();
                    }
                    else
                    {
                        set_vert_points = vert_points;
                        set_horiz_points = 0;
                        set_vert_lines = vert_lines;
                        set_horiz_lines = 0;
                    }
                    std::cout << std::endl
                              << "\033[0;35m[Venom Similator Printer] The settings was constructed. \033[0m"
                              << std::endl;

                    // set config parameters for env
                    GetUserSettings(set_traject_num, 
                                    traject_type, 
                                    set_vert_lines, 
                                    set_horiz_lines, 
                                    set_vert_points,
                                    set_horiz_points);
                    // detach those setting buttons
                    traject_num.Detach();
                    traject_type.Detach();
                    vert_points.Detach();
                    // horiz_points.Detach();
                    vert_lines.Detach();
                    // horiz_lines.Detach();
                    set_env.Detach();
                }

                if (start_traject_env && !click_build_env && click_set_start_means)
                {
                    menuShowTrajectory = true;
                    menuShowPoint = true;
                    menuShowLine = true;
                    std::cout << std::endl
                              << "\033[0;35m[Venom Similator Printer] Show environments and cameras. \033[0m"
                              << std::endl;
                    click_set_start_means = false;
                    // start signal for drawing env&pose
                    SetEnvironment(venom_settings); // SystemParameters();
                    start_traject_env.Detach();
                }

                {// draw ground truth environment
                    if (menuShowTrajectory)
                    {
                        std::vector<pangolin::OpenGlMatrix> MsTrue;
                        Eigen::Vector3d gt_color(1.0, 0.0, 0.0);
                        if (Twcs_true_.size() > 0)
                        {
                            DrawTrajectoryConnection(MsTrue, Twcs_true_, gt_color);
                            DrawTrajectoryCamera(MsTrue, Vec3(0, 1, 1));
                        }
                    }
                    if (menuShowLine)
                    {
                        Eigen::Vector3d gt_color(1.0, 0.0, 0.0);
                        DrawMapLine(lines_gt_, gt_color);
                    }
                    if (menuShowPoint)
                    {
                        Eigen::Vector3d gt_color(1.0, 0.0, 0.0);
                        DrawMapPoint(points_gt_, gt_color);
                    }
                }
                
                // 
                if (menuPrepareTracking && click_start_track && !click_set_start_means)
                {
                    //---> point_line
                    // camera-landmark association
                    ptr_tracker_ = new simulator::Track(
                        ptr_env_manager_->asso_epid_frameid_pos_, 
                        ptr_env_manager_->asso_epid_frameid_pixeld_,
                        ptr_env_manager_->asso_frameid_epid_, 
                        ptr_env_manager_->asso_elid_frameid_pos_,
                        ptr_env_manager_->asso_elid_frameid_pixeld_,
                        ptr_env_manager_->asso_frameid_elid_,
                        ptr_env_manager_->ptr_robot_trajectory_);

                    click_start_track = false;
                    ptr_tracker_->SaveFrameGTTrajectoryLovelyTUM(root_path_ + "ground_truth.txt");
                    std::vector<std::pair<int, Vec3>> vec_epid_pos_w;
                    vec_epid_pos_w.clear();
                    std::vector<std::pair<int, Mat32>> vec_elid_pos_w; // TODO: NULL
                    vec_elid_pos_w.clear();
                    ptr_env_manager_->GetMapLandmarks(vec_epid_pos_w, vec_elid_pos_w);
                    // for(int i =0; i<vec_epid_pos_w.size(); i++)
                    //     std::cout<<"ssss:"<<vec_epid_pos_w[i].second<<std::endl;
                    ptr_tracker_->SetMapLandmarks(vec_epid_pos_w);

                    std::cout
                        << std::endl
                        << "\033[0;35m[Venom Similator Printer] Associate landmarks to each camera. \033[0m"
                        << std::endl;

                    PageTrackPrivateDate();
                }

                pangolin::FinishFrame();
            }

            pangolin::QuitAll();
        }
        
        void PageTrackPrivateDate()
        {
            pangolin::CreatePanel("trackmenu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
            //
            pangolin::Var<bool> menuShowTrajectory("trackmenu.Show TrajectoryGT", false, true);
            pangolin::Var<bool> menuShowPoint("trackmenu.Groudtruth Point", false, true);
            pangolin::Var<bool> menuShowLine("trackmenu.Groudtruth Line", false, true);
            // ----------------------> Covisibility
            // tracking and mapping
            pangolin::Var<bool> menuLocalMapPointLine("trackmenu.Initial Pose Estimation and Recon.", false, false);
            // initial pose and landmarks
            pangolin::Var<bool> menuShowInitialPoses("trackmenu.Initial Poses", true, true);
            pangolin::Var<bool> menuShowInitialMapPoints("trackmenu.Initial MapPoints", true, true);
            pangolin::Var<bool> menuShowInitialMapLines("trackmenu.Initial MapLines", true, true);
            pangolin::Var<bool> start_ba("trackmenu.Optimization Preparation", false, false);
            // optimization
            // pangolin::Var<bool> menuPointBAGTSAM("trackmenu.CovisPointBA GTSAM", false, false);
            pangolin::Var<bool> menuPointBACeres("trackmenu.CovisPointBA", false, false);
            pangolin::Var<bool> menuPointLineBA("trackmenu.CovisPointLineBA", false, false);
            // pangolin::Var<bool> menuPointLineBACE("trackmenu.CovisExtensImpliPointLineBA", false, false);
            // pangolin::Var<bool> menuPointLineBACEEx("trackmenu.CovisExtensExpliPointLineBA", false, false);
            // refined pose and landmarks
            pangolin::Var<bool> menuShowOptimizedPoses("trackmenu.Optimized Poses", false, true);
            pangolin::Var<bool> menuShowOptimizedMapPoints("trackmenu.Optimized MapPoints", false, true);
            pangolin::Var<bool> menuShowOptimizedMapLines("trackmenu.Optimized MapLines", false, true);

            bool click_start_vo = true;
            bool click_get_initial_pose = true;
            bool click_start_opti = true;
            bool click_do_opti = true;

            std::map<int, Eigen::Matrix4d> Twcs_factorgraph;
            std::vector<Eigen::Matrix4d> optimized_poses;
            std::vector<Mat32> lines;
            std::vector<Vec3> points;

            std::vector<Eigen::Matrix4d> initial_poses;
            while (!pangolin::ShouldQuit())
            {
                // clear
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);

                { // ground truth visualization
                    if (menuShowTrajectory)
                    {
                        std::vector<pangolin::OpenGlMatrix> MsTrue;
                        Eigen::Vector3d gt_color(1.0, 0.0, 0.0);
                        if (Twcs_true_.size())
                        {
                            DrawTrajectoryConnection(MsTrue, Twcs_true_, gt_color);
                            DrawTrajectoryCamera(MsTrue, Vec3(0, 1, 1));
                        }
                    }
                    if (menuShowLine)
                    {
                        Eigen::Vector3d color(1.0, 0.0, 0.0);
                        DrawMapLine(lines_gt_, color);
                    }
                    if (menuShowPoint)
                    {
                        Eigen::Vector3d color(1.0, 0.0, 0.0);
                        DrawMapPoint(points_gt_, color);
                    }
                }

                if (click_start_vo)
                {
                    if (menuLocalMapPointLine) // TODO: Check
                    {
                        // ptr_tracker_->TrackOdometry(root_path_, true, true);
                        // initial_poses.clear();
                        std::thread *ptr_thread_ptr_tracker_ = new std::thread(&Track::TrackOdometryPtr, ptr_tracker_, root_path_, true, true, std::ref(initial_poses));
                        ptr_thread_ptr_tracker_->detach();

                        menuShowInitialMapPoints = true;
                        menuShowInitialMapLines = true;
                        menuShowInitialPoses = true;
                        menuLocalMapPointLine.Detach();
                        click_start_vo = false;
                    }
                    if (!click_start_vo)
                        std::cout
                            << std::endl
                            << "\033[0;35m[Venom Similator Printer] Finish pose estimation. \033[0m"
                            << std::endl;
                }

                if (!click_start_vo && click_get_initial_pose && ptr_tracker_->mbTrackOdometry && click_do_opti) // TODO: Check
                {
                    if (menuLocalMapPointLine)
                    {
                        // ptr_tracker_->GetInitFramePosesVect(initial_poses);

                        // Eigen::Vector3d color_initial_;
                        // std::vector<Vec3> points_initial_;

                        color_initial_ << 0.5, 1.0, 0.5;
                        std::vector<Vec3> points;
                        std::map<int /*mappoint_id*/, Vec3 /*posi_in_w*/> mappoints = ptr_tracker_->local_mappoints_;
                        for (auto &mit : mappoints)
                            points_initial_.push_back(mit.second);

                        color_pl_initial_ << 0.6, 0.9, 0.4;
                        std::map<int /*mapline_id*/, Mat32 /*posi_in_w*/> maplines = ptr_tracker_->local_maplines_;
                        for (auto mit = maplines.begin(); mit != maplines.end(); mit++)
                            lines_intial_.push_back(mit->second);

                        click_get_initial_pose = false;
                    }
                }

                if (menuShowInitialMapPoints)
                {
                    if (!click_get_initial_pose)
                        DrawMapPoint(points_initial_, color_initial_);
                    else
                    {
                        // show mappoints incrementally
                        points_initial_.clear();
                        color_initial_ << 0.5, 1.0, 0.5;
                        std::vector<Vec3> points;
                        std::map<int /*mappoint_id*/, Vec3 /*posi_in_w*/> mappoints = ptr_tracker_->local_mappoints_;
                        for (auto &mit : mappoints)
                            points_initial_.push_back(mit.second);

                        DrawMapPoint(points_initial_, color_initial_);
                    }
                }

                if (menuShowInitialMapLines)
                {
                    if (!click_get_initial_pose)
                        DrawMapLine(lines_intial_, color_pl_initial_);
                    else
                    {
                        lines_intial_.clear();
                        color_pl_initial_ << 0.6, 0.9, 0.4;
                        std::map<int /*mapline_id*/, Mat32 /*posi_in_w*/> maplines = ptr_tracker_->local_maplines_;
                        for (auto mit = maplines.begin(); mit != maplines.end(); mit++)
                            lines_intial_.push_back(mit->second);
                        DrawMapLine(lines_intial_, color_pl_initial_);
                    }
                }

                if (menuShowInitialPoses)
                {
                    std::vector<pangolin::OpenGlMatrix> MsTrue;
                    if (!click_get_initial_pose)
                    {
                        if (initial_poses.size())
                        {
                            DrawTrajectoryConnection(MsTrue, initial_poses, Vec3(0.4, 0.1, 0.0));
                            DrawTrajectoryCamera(MsTrue, Vec3(0.1, 0.8, 0.4));
                        }
                    }
                    else
                    {
                        // ptr_tracker_->GetInitFramePosesVect(initial_poses);
                        if (initial_poses.size() > 3)
                        {
                            DrawTrajectoryConnection(MsTrue, initial_poses, Vec3(0.4, 0.1, 0.0));
                            DrawTrajectoryCamera(MsTrue, Vec3(0.1, 0.8, 0.4));
                        }
                    }
                }

                if (menuShowOptimizedMapPoints)
                {
                    Eigen::Vector3d color(0.1, 0.5, 0.1);
                    // std::vector<Vec3> points;
                    // for (auto mit = optimized_mappoints_.begin(); mit != optimized_mappoints_.end(); mit++)
                    // {
                    //     points.push_back(mit->second);
                    // }
                    DrawMapPoint(points, color);
                }

                if (menuShowOptimizedMapLines)
                {
                    // DrawOptimizedMapLine();
                    Eigen::Vector3d color(0.1, 0.5, 0.1);
                    // std::vector<Mat32> lines;
                    // for (auto mit = optimized_maplines_.begin(); mit != optimized_maplines_.end(); mit++)
                    // {
                    //     lines.push_back(mit->second);
                    // }
                    DrawMapLine(lines, color);
                }

                if (menuShowOptimizedPoses)
                {
                    std::vector<pangolin::OpenGlMatrix> MsTrue;
                    // for (auto &tws : Twcs_factorgraph)
                    //     optimized_poses.push_back(tws.second);
                    if (optimized_poses.size() > 0)
                    {
                        DrawTrajectoryConnection(MsTrue, optimized_poses, Vec3(0.5, 0.5, 1));
                        DrawTrajectoryCamera(MsTrue, Vec3(0.1, 0.4, 0.8));
                    }
                }

                if (!click_start_vo && click_start_opti)
                {
                    // prepare data for optimization
                    if (start_ba)
                    {
                        ptr_tracker_->GetInitFramePoses(opti_para_.kfs);
                        // global position
                        ptr_tracker_->GetInitMapPoints(opti_para_.mappoints);
                        ptr_tracker_->GetInitMapLines(opti_para_.maplines);
                        ptr_env_manager_->GetAssoMPMeas(opti_para_.asso_mp_meas);
                        ptr_env_manager_->GetAssoFrameMPs(opti_para_.mp_obsers);
                        ptr_env_manager_->GetAssoMLMeas(opti_para_.asso_ml_meas);
                        ptr_env_manager_->GetAssoMLMeasD(opti_para_.asso_ml_posi);
                        ptr_env_manager_->GetAssoParalis(opti_para_.asso_paralineid_mlids);
                        ptr_env_manager_->GetAssoFrameMLs(opti_para_.mp_obsers);
                        click_start_opti = false;
                        // record
                        IO::RecordPrivateData(root_path_, opti_para_);
                        start_ba.Detach();
                    }
                }

                if (!click_start_opti && click_do_opti)
                {
                    if (menuPointBACeres)
                    {
                        simulator::optimizer::CovisPointBA::optimizer(ptr_robot_trajectory_,
                                                                      opti_para_,
                                                                      optimized_mappoints_,
                                                                      Twcs_factorgraph);
                        menuShowOptimizedPoses = true;
                        menuShowOptimizedMapPoints = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;

                        click_do_opti = false;
                        for (auto &tws : Twcs_factorgraph)
                            optimized_poses.push_back(tws.second);
                        // for (auto mit = optimized_maplines_.begin(); mit != optimized_maplines_.end(); mit++)
                        // {
                        //     lines.push_back(mit->second);
                        // }
                        for (auto mit = optimized_mappoints_.begin(); mit != optimized_mappoints_.end(); mit++)
                        {
                            points.push_back(mit->second);
                        }
                        // menuPointBAGTSAM.Detach();
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                        // menuPointLineBACE.Detach();
                        // menuPointLineBACEEx.Detach();
                    }
                    else if (menuPointLineBA)
                    {

                        simulator::optimizer::CovisPointLineBA::optimizer(ptr_robot_trajectory_,
                                                                          opti_para_,
                                                                          optimized_mappoints_,
                                                                          optimized_maplines_,
                                                                          Twcs_factorgraph);
                        menuShowOptimizedPoses = true;
                        menuShowOptimizedMapPoints = true;
                        menuShowOptimizedMapLines = true;
                        std::cout << std::endl
                                  << "\033[0;35m[Venom Similator Printer] Optimize reconstructed landmarks and "
                                     "cameras. \033[0m"
                                  << std::endl;
                        click_do_opti = false;
                        for (auto &tws : Twcs_factorgraph)
                            optimized_poses.push_back(tws.second);
                        for (auto mit = optimized_maplines_.begin(); mit != optimized_maplines_.end(); mit++)
                        {
                            lines.push_back(mit->second);
                        }
                        for (auto mit = optimized_mappoints_.begin(); mit != optimized_mappoints_.end(); mit++)
                        {
                            points.push_back(mit->second);
                        }
                        menuPointBACeres.Detach();
                        menuPointLineBA.Detach();
                    }
                }
                pangolin::FinishFrame();
            }
        }

    private:
        /**
         * @brief Set the System Parameters based on user's inputs
         *
         * @param traject_num
         * @param traject_type
         * @param vert_lines
         * @param horiz_lines
         * @param vert_points
         * @param horiz_points
         */
        void GetUserSettings(const int &traject_num, const int &traject_type, const int &vert_lines,
                             const int &horiz_lines, const int &vert_points, const int &horiz_points)
        {
            env_para_.traject_type_ = traject_type;
            env_para_.frame_num_ = traject_num;
            env_para_.vert_points_ = vert_points;
            env_para_.horiz_points_ = 0;
            env_para_.vert_lines_ = vert_lines;
            env_para_.horiz_lines_ = 0;
            // save config
            IO::SaveSystemConfig(root_path_ + "system_parameter.txt", env_para_);
            std::cout << std::endl
                      << "\033[0;35m[Venom Similator Printer] Parameters are saved in the system_parameter file. \033[0m" << std::endl;
        }

        /**
         * @brief Set the Environment object: Ground truth
         *
         * @param settings
         */
        void SetEnvironment(cv::FileStorage &settings)
        {
            // initialize EnvManager
            ptr_env_manager_ = new simulator::EnvManager(env_para_, settings);
            // initialize trajectory
            ptr_robot_trajectory_ = ptr_env_manager_->ptr_robot_trajectory_;
            for (auto frame_info : ptr_robot_trajectory_->groundtruth_traject_id_Twc_)
            {
                // int frame_idx = frame_info.first;
                // for drawing
                Twcs_true_.push_back(frame_info.second);
            }

            bool is_public = false;
            settings["Env.Public.bool"] >> is_public;

            if (is_public)
            {
                // std::cout << "read public 0" << std::endl;
                ptr_env_manager_->BuildPublicPoints();
                // std::cout << "read public" << std::endl;
                ptr_env_manager_->GetEnvPoints(points_gt_); // for visualizaiton
                // std::cout << "read public 2" << std::endl;
                ptr_env_manager_->BuildPublicLines();
                ptr_env_manager_->GetEnvLines(lines_gt_); // for visualizaiton
            }
            else
            {
                // env_points
                ptr_env_manager_->BuildEnvPoints();         // build ground truth points
                ptr_env_manager_->GetEnvPoints(points_gt_); // for visualizaiton
                // env_lines
                ptr_env_manager_->BuildEnvLines();        // build ground truth lines
                ptr_env_manager_->GetEnvLines(lines_gt_); // for visualizaiton
            }
        }

        /**
         * @brief
         *
         * @param lines
         * @param color
         */
        void DrawMapLine(std::vector<Mat32> &lines, Eigen::Vector3d &color)
        {
            glLineWidth(4.0);
            glBegin(GL_LINES);
            glColor3f(color(0), color(1), color(2));
            for (const auto &Line : lines)
            {
                Vec3 line0 = Line.block(0, 0, 3, 1);
                Vec3 line1 = Line.block(0, 1, 3, 1);
                glVertex3f(line0(0), line0(1), line0(2));
                glVertex3f(line1(0), line1(1), line1(2));
            }
            glEnd();
        }

        /**
         * @brief Draw map points
         *
         * @param points
         * @param color
         */
        void DrawMapPoint(std::vector<Vec3> &points, Eigen::Vector3d &color)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            glColor3f(color(0), color(1), color(2));

            for (auto p : points)
                glVertex3d(p[0], p[1], p[2]);
            glEnd();
        }

        /**
         * @brief
         *
         * @param Ms
         * @param Twcs_true
         * @param color
         */
        void DrawTrajectoryConnection(std::vector<pangolin::OpenGlMatrix> &Ms, std::vector<Mat4> &Twcs_true, Vec3 color = Vec3(0, 1, 0))
        {
            {
                Ms.clear();
                for (auto &Twc : Twcs_true)
                {
                    pangolin::OpenGlMatrix M;

                    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
                    Vec3 twc = Twc.block(0, 3, 3, 1);

                    M.m[0] = Rwc(0, 0);
                    M.m[1] = Rwc(1, 0);
                    M.m[2] = Rwc(2, 0);
                    M.m[3] = 0.0;

                    M.m[4] = Rwc(0, 1);
                    M.m[5] = Rwc(1, 1);
                    M.m[6] = Rwc(2, 1);
                    M.m[7] = 0.0;

                    M.m[8] = Rwc(0, 2);
                    M.m[9] = Rwc(1, 2);
                    M.m[10] = Rwc(2, 2);
                    M.m[11] = 0.0;

                    M.m[12] = twc(0);
                    M.m[13] = twc(1);
                    M.m[14] = twc(2);
                    M.m[15] = 1.0;

                    Ms.push_back(M);
                }
            }
            glLineWidth(2);
            glBegin(GL_LINES);
            glColor3f(color(0), color(1), color(2));
            for (int i = 0; i < Twcs_true.size() - 2; i++)
            {
                glVertex3f(Twcs_true[i](0, 3), Twcs_true[i](1, 3), Twcs_true[i](2, 3));
                glVertex3f(Twcs_true[i + 1](0, 3), Twcs_true[i + 1](1, 3), Twcs_true[i + 1](2, 3));
            }
            glEnd();
        }

        /**
         * @brief
         *
         * @param connections
         */
        void DrawTrajectoryConnection(std::vector<Vec3> &connections)
        {
            glLineWidth(2);
            for (int j = 0; j < connections.size(); j++)
            {
                // std::cout<<"venom_association_[i]:"<<venom_association_[j].size()<<std::endl;
                int edge_type = connections[j](0); // 0, 1

                assert(edge_type >= 0);
                int vertex_1 = connections[j](1);
                int vertex_2 = connections[j](2);

                glBegin(GL_LINES);
                int color = 0.2 + edge_type * edge_type / 4; //
                glColor3f(0.5f * color, 1.f * color, 0.1f * color);

                glVertex3f(Twcs_true_[vertex_2](0, 3),
                           Twcs_true_[vertex_2](1, 3),
                           Twcs_true_[vertex_2](2, 3));
                glVertex3f(Twcs_true_[vertex_1](0, 3),
                           Twcs_true_[vertex_1](1, 3),
                           Twcs_true_[vertex_1](2, 3));
            }
            glEnd();
        }

        /**
         * @brief
         *
         * @param M
         * @param r
         * @param g
         * @param b
         */
        void DrawSigCam(pangolin::OpenGlMatrix &M, GLfloat &r, GLfloat &g, GLfloat &b)
        {
            // camera size
            const float &w = 0.1;
            const float h = w * 0.638;
            const float z = w * 0.6;

            glPushMatrix();

#ifdef HAVE_GLES
            glMultMatrixf(M.m);
#else
            glMultMatrixd(M.m);
#endif
            glLineWidth(1.0);
            glColor3f(r, g, b);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();
        }

        /**
         * @brief
         *
         * @param Ms
         * @param color
         */
        void DrawTrajectoryCamera(std::vector<pangolin::OpenGlMatrix> &Ms, Vec3 color = Vec3(0, 0, 1))
        {
            for (auto &M : Ms)
            {
                GLfloat r = color.x(); // 0.0f;
                GLfloat g = color.y(); // 1.0f;
                GLfloat b = color.z(); // 0.0f;
                DrawSigCam(M, r, g, b);
            }
        }
    };
}

#endif //__VENOM_SRC_VISUALIZER_INTERFACE_HPP__