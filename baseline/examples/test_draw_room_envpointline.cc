/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-27 08:08:06
 * @LastEditTime: 2023-08-27 15:39:54
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /JointFactorGraph/examples/test_draw_room_envpointline.cc
 */

#include <iostream>

#include "../src/utils/UtilEnvElement.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <random>
#include <vector>

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

void SpherePoseGeneration(double radius,
                          int nums_points,
                          int index,
                          std::vector<Eigen::Matrix4d> &vec_traject_gt_Twc)
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
    vec_traject_gt_Twc.push_back(Twc);
}

bool GenerateSphereKeyFrames(const int frame_num,
                             double radius,
                             std::vector<Eigen::Matrix4d> &vec_traject_gt_Twc)
{
    if (frame_num < 2)
        return false;
    for (int n = 0; n < frame_num; n++)
    {
        double radius = 2.0;
        // double theta = n * 2. * M_PI / frame_num;
        SpherePoseGeneration(radius, frame_num, n, vec_traject_gt_Twc);
    }
}

void DrawTrajectoryCamera(std::vector<pangolin::OpenGlMatrix> &Ms, Eigen::Vector3d color = Eigen::Vector3d(0, 0, 1))
{
    for (auto &M : Ms)
    {
        GLfloat r = color.x(); // 0.0f;
        GLfloat g = color.y(); // 1.0f;
        GLfloat b = color.z(); // 0.0f;
        DrawSigCam(M, r, g, b);
    }
}

void AddScenePart(std::map<std::string, std::vector<Eigen::Vector3d>> &cube, Eigen::Vector3d &color)
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glColor3f(color(0), color(1), color(2));
    for (auto plane : cube)
        // if(plane.first == "bottom" || plane.first == "top" )
        for (size_t i = 0; i < plane.second.size() - 1; i++)
        {
            if (i == 0)
            {
                glVertex3f(plane.second[i](0), plane.second[i](1), plane.second[i](2));
                glVertex3f(plane.second[i + 3](0), plane.second[i + 3](1), plane.second[i + 3](2));
            }
            glVertex3f(plane.second[i](0), plane.second[i](1), plane.second[i](2));
            glVertex3f(plane.second[i + 1](0), plane.second[i + 1](1), plane.second[i + 1](2));
        }
    glEnd();
}
void AddDenseScenePart(std::map<std::string, std::vector<Eigen::Vector3d>> &cube, Eigen::Vector3d &color)
{
    glLineWidth(4.0);
    glBegin(GL_QUADS);
    glColor3f(color(0), color(1), color(2));
    for (auto plane : cube)
    {
        glVertex3f(plane.second[0](0), plane.second[0](1), plane.second[0](2));
        glVertex3f(plane.second[1](0), plane.second[1](1), plane.second[1](2));
        glVertex3f(plane.second[2](0), plane.second[2](1), plane.second[2](2));
        glVertex3f(plane.second[3](0), plane.second[3](1), plane.second[3](2));
    }
    glEnd();
}

void DrawTrajectoryConnection(std::vector<pangolin::OpenGlMatrix> &Ms,
                              std::vector<Eigen::Matrix4d> &Twcs_true,
                              Eigen::Vector3d color = Eigen::Vector3d(0, 1, 0))
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

void DrawMapLine(std::vector<Eigen::Matrix<double, 3, 2>> &lines, Eigen::Vector3d &color)
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    glColor3f(color(0), color(1), color(2));
    for (const auto &Line : lines)
    {
        Eigen::Vector3d line0 = Line.block(0, 0, 3, 1);
        Eigen::Vector3d line1 = Line.block(0, 1, 3, 1);
        glVertex3f(line0(0), line0(1), line0(2));
        glVertex3f(line1(0), line1(1), line1(2));
    }
    glEnd();
}

void DrawCoordiante()
{
    glLineWidth(4.0);
    glBegin(GL_LINES);
    Eigen::Matrix3d coord = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++)
    {
        // r g b
        glColor3f(255 * coord(i, 0), coord(i, 1) * 255, coord(i, 2) * 255);
        glVertex3f(0, 0, 0);
        glVertex3f(coord(i, 0), coord(i, 1), coord(i, 2));
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
    std::uniform_real_distribution<double> width_point_generate(plane_min_bounds[0], plane_max_bounds[0]);   // width distribution
    std::uniform_real_distribution<double> lenghth_point_generate(plane_min_bounds[1], plane_max_bounds[1]); // height distribution
    std::uniform_real_distribution<double> height_point_generate(plane_min_bounds[2], plane_max_bounds[2]);  // height distribution
    pos_world << width_point_generate(generator), lenghth_point_generate(generator), height_point_generate(generator);
}

void GenerateEnvLine(const Eigen::Vector3d &plane_max_bounds,
                     const Eigen::Vector3d &plane_min_bounds,
                     Eigen::Matrix<double, 3, 2> &endpos_world)
{
    std::random_device rd;
    std::default_random_engine generator(rd());

    int dominent_direct = -1;
    int plane_direc = -1;
    double max_distance = -10000.0;
    double min_distance = 10000.0;
    std::map<int, double> ep_value;
    std::map<int, double> sp_value;
    for (int i = 0; i < 3; i++)
    {
        double i_aix = std::abs(plane_max_bounds[i] - plane_min_bounds[i]);
        if (i_aix > max_distance)
        {
            dominent_direct = i;
            max_distance = i_aix;
        }
        if (i_aix < min_distance)
        {
            plane_direc = i;
            min_distance = i_aix;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (i == dominent_direct || i == plane_direc)
            continue;

        std::uniform_real_distribution<double> support_point_generate(plane_min_bounds[i], plane_max_bounds[i]); // width distribution
        double support = support_point_generate(generator);

        ep_value[i] = support;
        sp_value[i] = support;
    }

    std::uniform_real_distribution<double> dominament_sp_generate(plane_min_bounds[dominent_direct], plane_max_bounds[dominent_direct]); // width distribution
    std::uniform_real_distribution<double> dominament_ep_generate(plane_min_bounds[dominent_direct], plane_max_bounds[dominent_direct]); // width distribution
    std::uniform_real_distribution<double> plane_point_generate(plane_min_bounds[plane_direc], plane_max_bounds[plane_direc]);           // width distribution
    double sp = dominament_sp_generate(generator);
    double ep = dominament_ep_generate(generator);
    ep_value[dominent_direct] = ep;
    sp_value[dominent_direct] = sp;

    double plane_p = plane_point_generate(generator);
    ep_value[plane_direc] = plane_p;
    sp_value[plane_direc] = plane_p;

    endpos_world.col(0) << sp_value[0], sp_value[1], sp_value[2];
    endpos_world.col(1) << ep_value[0], ep_value[1], ep_value[2];
}

void GetBoundsOfPlane(std::vector<Eigen::Vector3d> &vertices, Eigen::Vector3d &max_bounds, Eigen::Vector3d &min_bounds)
{
    std::vector<double> vec_vertex_x;
    std::vector<double> vec_vertex_y;
    std::vector<double> vec_vertex_z;

    for (auto vertex : vertices)
    {
        vec_vertex_x.push_back(vertex.x());
        vec_vertex_y.push_back(vertex.y());
        vec_vertex_z.push_back(vertex.z());
    }

    auto get_bound = [&](std::vector<double> &vec_vertex_x,
                         std::vector<double> &vec_vertex_y,
                         std::vector<double> &vec_vertex_z, bool is_max = true)
    {
        Eigen::Vector3d vec_bound;
        if (is_max)
        {
            double max_x = *std::max_element(vec_vertex_x.begin(), vec_vertex_x.end());
            double max_y = *std::max_element(vec_vertex_y.begin(), vec_vertex_y.end());
            double max_z = *std::max_element(vec_vertex_z.begin(), vec_vertex_z.end());
            vec_bound << max_x, max_y, max_z;
        }
        else
        {
            double min_x = *std::min_element(vec_vertex_x.begin(), vec_vertex_x.end());
            double min_y = *std::min_element(vec_vertex_y.begin(), vec_vertex_y.end());
            double min_z = *std::min_element(vec_vertex_z.begin(), vec_vertex_z.end());
            vec_bound << min_x, min_y, min_z;
        }
        return vec_bound;
    };
    max_bounds = get_bound(vec_vertex_x, vec_vertex_y, vec_vertex_z, true);
    min_bounds = get_bound(vec_vertex_x, vec_vertex_y, vec_vertex_z, false);
}

void BuildRoom(std::map<std::string, std::vector<Eigen::Vector3d>> &cube_top_planes,
               std::map<std::string, std::vector<Eigen::Vector3d>> &cube_right_planes,
               std::map<std::string, std::vector<Eigen::Vector3d>> &cube_bottom_planes,
               std::map<std::string, std::vector<Eigen::Vector3d>> &cube_left_planes,
               std::vector<Eigen::Vector3d> &env_mappoints,
               std::vector<Eigen::Matrix<double, 3, 2>> &env_maplines)
{
    // get all planes
    std::vector<std::vector<Eigen::Vector3d /*plane_vertex*/>> planes;
    for (auto plane : cube_top_planes)
    {
        std::cout << "plane_id:" << plane.first << std::endl;
        planes.push_back(plane.second);
    }
    for (auto plane : cube_right_planes)
    {
        std::cout << "plane_id:" << plane.first << std::endl;
        planes.push_back(plane.second);
    }
    for (auto plane : cube_bottom_planes)
    {
        std::cout << "plane_id:" << plane.first << std::endl;
        planes.push_back(plane.second);
    }
    for (auto plane : cube_left_planes)
    {
        std::cout << "plane_id:" << plane.first << std::endl;
        planes.push_back(plane.second);
    }

    // generate points
    for (int id = 0; id < 500 /*500 map points*/; id++)
    {
        int i = id % planes.size();

        Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
        Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
        GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
        std::cout << "plane_max_min:" << plane_max_bounds << "," << plane_min_bounds << std::endl;
        Eigen::Vector3d env_point_i;
        GenerateEnvPoint(plane_max_bounds, plane_min_bounds, env_point_i);
        std::cout << ">> generated:" << env_point_i << std::endl;
        env_mappoints.push_back(env_point_i);
    }
    std::cout << "map size:" << env_mappoints.size() << std::endl;

    // generate points
    for (int id = 0; id < 100 /*500 map points*/; id++)
    {
        int i = id % planes.size();

        Eigen::Vector3d plane_max_bounds = Eigen::Vector3d::Zero();
        Eigen::Vector3d plane_min_bounds = Eigen::Vector3d::Zero();
        GetBoundsOfPlane(planes[i], plane_max_bounds, plane_min_bounds);
        std::cout << "plane_max_min:" << plane_max_bounds << "," << plane_min_bounds << std::endl;
        Eigen::Matrix<double, 3, 2> env_line_i;
        GenerateEnvLine(plane_max_bounds, plane_min_bounds, env_line_i);
        std::cout << ">> generated:" << env_line_i << std::endl;
        env_maplines.push_back(env_line_i);
    }
}

int main(/*int argc, char* argv[]*/)
{
    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main", 640, 480);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.7, 10.8, 0, 0, -50.0, 0.0, 10.0, 10.0));

    // Choose a sensible left UI Panel width based on the width of 20
    // charectors from the default font.
    const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 640.0f / 480.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    glClearColor(0.1 * 10, 0.1 * 10, 0.1 * 10, 0.1 * 10);

    // Safe and efficient binding of named variables.
    // Specialisations mean no conversions take place for exact types
    // and conversions between scalar types are cheap.
    pangolin::Var<bool> a_button("ui.A_Button", false, false);
    pangolin::Var<double> a_double("ui.A_Double", 3, 0, 5);
    pangolin::Var<int> an_int("ui.An_Int", 2, 0, 5);
    pangolin::Var<double> a_double_log("ui.Log_scale", 3, 1, 1E4, true);
    pangolin::Var<bool> a_checkbox("ui.A_Checkbox", false, true);
    pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input", 2);
    pangolin::Var<std::string> a_string("ui.A_String", "Edit ME!");

    // std::function objects can be used for Var's too. These work great with C++11 closures.
    pangolin::Var<std::function<void(void)>> save_window("ui.Save_Window", []()
                                                         { pangolin::SaveWindowOnRender("window"); });

    pangolin::Var<std::function<void(void)>> save_cube_view("ui.Save_Cube", [&d_cam]()
                                                            { pangolin::SaveWindowOnRender("cube", d_cam.v); });

    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', [&]()
                                       { a_double = 3.5; });

    double length = 8.0, width = 7, height = 5;
    Room room_env = Room(length, width, height);
    std::map<std::string, std::vector<Eigen::Vector3d>> room_planes;
    std::map<std::string, std::vector<Eigen::Vector3d>> picture_planes;
    std::map<std::string, std::vector<Eigen::Vector3d>> cabinet_planes;
    std::map<std::string, std::vector<Eigen::Vector3d>> fridge_planes;
    room_env.GetRegions(room_planes, picture_planes, cabinet_planes, fridge_planes);

    std::vector<Eigen::Vector3d> env_mappoints;
    std::vector<Eigen::Matrix<double, 3, 2>> env_maplines;
    BuildRoom(room_planes,
              picture_planes,
              cabinet_planes,
              fridge_planes,
              env_mappoints,
              env_maplines);

    // Default hooks for exiting (Esc) and fullscreen (tab).
    // geneerate trajectory
    std::vector<Eigen::Matrix4d> vec_traject_gt_Twc;
    GenerateSphereKeyFrames(100, 9, vec_traject_gt_Twc);

    Eigen::Vector3d color(0.1, 0.5, 0.1);
    Eigen::Vector3d frame_line_color(0.5, 0.1, 0.1);
    Eigen::Vector3d frame_line_color_2(0.1, 0.1, 0.4);
    Eigen::Vector3d frame_line_color_3(0.1, 0.3, 0.4);
    Eigen::Vector3d frame_line_color_4(0.1, 0.1, 0.1);
    while (!pangolin::ShouldQuit())
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (pangolin::Pushed(a_button))
            std::cout << "You Pushed a button!" << std::endl;

        // Overloading of Var<T> operators allows us to treat them like
        // their wrapped types, eg:
        if (a_checkbox)
            an_int = (int)a_double;

        an_int_no_input = an_int;

        if (d_cam.IsShown())
        {
            // Activate efficiently by object
            d_cam.Activate(s_cam);
            // Render some stuff
            // glColor3f(1.0,1.0,1.0);
            // pangolin::glDrawColouredCube();
            AddScenePart(room_planes, frame_line_color);
            AddDenseScenePart(picture_planes, frame_line_color_3);
            AddDenseScenePart(cabinet_planes, frame_line_color_3);
            AddDenseScenePart(fridge_planes, frame_line_color_4);
            DrawOrigin();
            DrawCoordiante();

            DrawMapPoint(env_mappoints, color);
            DrawMapLine(env_maplines, color);

            std::vector<pangolin::OpenGlMatrix> MsTrue;

            DrawTrajectoryConnection(MsTrue, vec_traject_gt_Twc, Eigen::Vector3d(0, 1, 1));
            DrawTrajectoryCamera(MsTrue, Eigen::Vector3d(0.1, 0.8, 0.4));
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}