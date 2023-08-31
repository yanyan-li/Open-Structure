/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-06 08:47:17
 * @LastEditTime: 2023-08-12 08:29:51
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description: 
 * @FilePath: /JointFactorGraph/examples/test_draw_corridor.cc
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

#include "../src/utils/UtilEnvElement.hpp"


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

void DrawOrigin()
{
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(0, 255, 0);
    glVertex3d(0, 0, 0);
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
  
  double length=20.0, width =18, height=3.5;
  Corridor corridor_env = Corridor(length, width, height);
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_top_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_left_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_bottom_planes;
  std::map<std::string, std::vector<Eigen::Vector3d>> cube_right_planes;
  corridor_env.GetRegions(cube_top_planes, cube_right_planes, cube_bottom_planes, cube_left_planes);

  // Default hooks for exiting (Esc) and fullscreen (tab).
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

        Eigen::Vector3d color(0.1, 0.5, 0.1);
        AddCorridorPart(cube_top_planes, color);
        AddCorridorPart(cube_right_planes, color);
        AddCorridorPart(cube_left_planes, color);
        AddCorridorPart(cube_bottom_planes, color);
        DrawOrigin();
        DrawCoordiante();
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
