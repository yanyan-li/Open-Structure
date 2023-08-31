/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-12 09:17:51
 * @LastEditTime: 2023-08-12 10:09:44
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /JointFactorGraph/examples/test_draw_corridor_env.cc
 */


#include <iostream>

#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <random>
#include "../src/utils/UtilEnvElement.hpp" 



void DrawSigCam(pangolin::OpenGlMatrix &M, GLfloat &r, GLfloat &g, GLfloat &b)
{
    // camera size
    const float &w = 0.3;
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

bool GenerateCorridorTrajectory(const int frame_num, double radius, double length_half, 
                         std::vector<Eigen::Matrix4d> &vec_traject_gt_Twc)
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

        vec_traject_gt_Twc.push_back(Twc);
    }

}

void DrawTrajectoryCamera(std::vector<pangolin::OpenGlMatrix> &Ms, Eigen::Vector3d color=Eigen::Vector3d(0,0,1)  )
{
    for (auto &M : Ms)
    {
        GLfloat r = color.x(); // 0.0f;
        GLfloat g = color.y(); // 1.0f;
        GLfloat b = color.z(); // 0.0f;
        DrawSigCam(M, r, g, b);
    }
}

void AddCorridorPart(std::map<std::string, std::vector<Eigen::Vector3d>> &cube, Eigen::Vector3d &color)
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glColor3f(color(0), color(1), color(2));
    for(auto plane: cube)
        // if(plane.first == "bottom" || plane.first == "top" )
            for(size_t i = 0; i< plane.second.size()-1; i++)
            {
                if(i==0)
                {
                    glVertex3f(plane.second[i](0), plane.second[i](1), plane.second[i](2));
                    glVertex3f(plane.second[i+3](0), plane.second[i+3](1), plane.second[i+3](2));
                }
                glVertex3f(plane.second[i](0), plane.second[i](1), plane.second[i](2));
                glVertex3f(plane.second[i+1](0), plane.second[i+1](1), plane.second[i+1](2));
            }
    glEnd();
}

void DrawTrajectoryConnection(std::vector<pangolin::OpenGlMatrix> &Ms, 
                             std::vector<Eigen::Matrix4d> &Twcs_true, 
                             Eigen::Vector3d color = Eigen::Vector3d(0,1,0))
{
    {
        Ms.clear();
        for (auto &Twc : Twcs_true)
        {
            pangolin::OpenGlMatrix M;

            Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
            Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);

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

void DrawOrigin()
{
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(0, 255, 0);
    glVertex3d(0, 0, 0);
    glEnd();
}
void DrawMapPoint(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &color)
{
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(color(0), color(1), color(2));

    for (auto p : points)
        glVertex3d(p[0], p[1], p[2]);
    glEnd();
}

void DrawCoordiante()
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    Eigen::Matrix3d coord = Eigen::Matrix3d::Identity();
    for(int i=0; i<3; i++)
    {
      // r g b
      glColor3f(255*coord(i,0), coord(i,1)*255, coord(i,2)*255);
      glVertex3f(0, 0, 0);
      glVertex3f(coord(i,0),coord(i,1), coord(i,2) );
    }
    glEnd();
}

void GenerateEnvPoint(const Eigen::Vector3d &plane_max_bounds, 
                     const Eigen::Vector3d &plane_min_bounds,
                     Eigen::Vector3d &pos_world)
{
    // std::default_random_engine generator;
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> width_point_generate(plane_min_bounds[0], plane_max_bounds[0]); // width distribution
    std::uniform_real_distribution<double> lenghth_point_generate(plane_min_bounds[1], plane_max_bounds[1]); // height distribution
    std::uniform_real_distribution<double> height_point_generate(plane_min_bounds[2], plane_max_bounds[2]); // height distribution
    pos_world <<  width_point_generate(generator),  lenghth_point_generate(generator),  height_point_generate(generator);
}

void GetBoundsOfPlane(std::vector<Eigen::Vector3d> & vertices, Eigen::Vector3d & max_bounds,  Eigen::Vector3d & min_bounds )
{
    std::vector<double> vec_vertex_x;
    std::vector<double> vec_vertex_y;
    std::vector<double> vec_vertex_z;
    
    for(auto vertex: vertices)
    {
        vec_vertex_x.push_back(vertex.x());
        vec_vertex_y.push_back(vertex.y());
        vec_vertex_z.push_back(vertex.z());
    }

    auto get_bound = [&](std::vector<double> &vec_vertex_x, 
                            std::vector<double> &vec_vertex_y,
                            std::vector<double> &vec_vertex_z, bool is_max=true ){
        Eigen::Vector3d vec_bound;
        if(is_max)
        {
            double max_x = *std::max_element(vec_vertex_x.begin(), vec_vertex_x.end());
            double max_y = *std::max_element(vec_vertex_y.begin(), vec_vertex_y.end());
            double max_z = *std::max_element(vec_vertex_z.begin(), vec_vertex_z.end());
            vec_bound <<max_x, max_y, max_z;
        }
        else
        {
            double min_x = *std::min_element(vec_vertex_x.begin(), vec_vertex_x.end());
            double min_y = *std::min_element(vec_vertex_y.begin(), vec_vertex_y.end());
            double min_z = *std::min_element(vec_vertex_z.begin(), vec_vertex_z.end());
            vec_bound <<min_x, min_y, min_z;
        }                    
        return vec_bound;
    };
    max_bounds = get_bound(vec_vertex_x,vec_vertex_y, vec_vertex_z, true);
    min_bounds = get_bound(vec_vertex_x,vec_vertex_y, vec_vertex_z, false);
}

void BuildCorridor(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_top_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_right_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_bottom_planes,
                    std::map<std::string, std::vector<Eigen::Vector3d>> &cube_left_planes,
                    std::vector<Eigen::Vector3d> &env_mappoints)
{   
    // get all planes
    std::vector< std::vector<Eigen::Vector3d/*plane_vertex*/>> planes;
    for( auto plane: cube_top_planes)
    {
        std::cout<<"plane_id:"<<plane.first<<std::endl;
        planes.push_back(plane.second);    
    }
    for( auto plane:cube_right_planes)
    {
        std::cout<<"plane_id:"<<plane.first<<std::endl;
        planes.push_back(plane.second);    
    }
    for( auto plane:cube_bottom_planes)
    {
        std::cout<<"plane_id:"<<plane.first<<std::endl;
        planes.push_back(plane.second);    
    }
    for( auto plane:cube_left_planes)
    {
        std::cout<<"plane_id:"<<plane.first<<std::endl;
        planes.push_back(plane.second);    
    }

    // 
    for(int id = 0; id < 500/*500 map points*/; id++)
    {
        int i = id%planes.size();
        
        Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
        Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
        GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
        std::cout<<"plane_max_min:"<<plane_max_bounds<<","<<plane_min_bounds<<std::endl;
        Eigen::Vector3d env_point_i;
        GenerateEnvPoint(plane_max_bounds, plane_min_bounds, env_point_i); 
        std::cout<<">> generated:"<<env_point_i<<std::endl;

        env_mappoints.push_back(env_point_i);
    }
    std::cout<<"map size:"<<env_mappoints.size()<<std::endl;
}


int main(/*int argc, char* argv[]*/)
{  
  // Create OpenGL window in single line
  pangolin::CreateWindowAndBind("Main",640,480);
  
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.7, 10.8, 0, 0, -50.0, 0.0, 10.0, 10.0));

  // Choose a sensible left UI Panel width based on the width of 20
  // charectors from the default font.
  const int UI_WIDTH = 20* pangolin::default_font().MaxWidth();

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  pangolin::Var<bool> a_button("ui.A_Button",false,false);
  pangolin::Var<double> a_double("ui.A_Double",3,0,5);
  pangolin::Var<int> an_int("ui.An_Int",2,0,5);
  pangolin::Var<double> a_double_log("ui.Log_scale",3,1,1E4, true);
  pangolin::Var<bool> a_checkbox("ui.A_Checkbox",false,true);
  pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input",2);
  pangolin::Var<std::string> a_string("ui.A_String", "Edit ME!");

  // std::function objects can be used for Var's too. These work great with C++11 closures.
  pangolin::Var<std::function<void(void)>> save_window("ui.Save_Window", [](){
      pangolin::SaveWindowOnRender("window");
  });

  pangolin::Var<std::function<void(void)>> save_cube_view("ui.Save_Cube", [&d_cam](){
      pangolin::SaveWindowOnRender("cube", d_cam.v);
  });

  // Demonstration of how we can register a keyboard hook to alter a Var
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', [&](){
      a_double = 3.5;
  });
  
  double length=28.0, width =27, height=3.5;
  Corridor corridor_env = Corridor(length, width, height);
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_top_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_left_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_bottom_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_right_planes;
  corridor_env.GetRegions(cube_top_planes, cube_right_planes, cube_bottom_planes, cube_left_planes);
  
  std::vector<Eigen::Vector3d> env_mappoints;
  BuildCorridor(cube_top_planes,
                cube_left_planes, 
                cube_bottom_planes,
                cube_right_planes, 
                env_mappoints);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  // geneerate trajectory
  std::vector<Eigen::Matrix4d> vec_traject_gt_Twc;
  GenerateCorridorTrajectory(100, 3, 9, vec_traject_gt_Twc);
  
  Eigen::Vector3d color(0.1, 0.5, 0.1);
  while( !pangolin::ShouldQuit())
  {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

    if( pangolin::Pushed(a_button) )
      std::cout << "You Pushed a button!" << std::endl;

    // Overloading of Var<T> operators allows us to treat them like
    // their wrapped types, eg:
    if( a_checkbox )
      an_int = (int)a_double;

    an_int_no_input = an_int;

    if(d_cam.IsShown()) {
        // Activate efficiently by object
        d_cam.Activate(s_cam);

        // Render some stuff
        // glColor3f(1.0,1.0,1.0);
        // pangolin::glDrawColouredCube();
        AddCorridorPart(cube_top_planes, color);
        AddCorridorPart(cube_right_planes, color);
        AddCorridorPart(cube_left_planes, color);
        AddCorridorPart(cube_bottom_planes, color);
        DrawOrigin();
        DrawCoordiante();

        DrawMapPoint(env_mappoints, color);
       
        std::vector<pangolin::OpenGlMatrix> MsTrue;
    
        DrawTrajectoryConnection(MsTrue, vec_traject_gt_Twc, Eigen::Vector3d(0,1,1));
        DrawTrajectoryCamera(MsTrue, Eigen::Vector3d(0.1, 0.8, 0.4));
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}