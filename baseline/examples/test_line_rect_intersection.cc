/*
 * @Author: yanyan-li yanyan.li.camp@gmail.com
 * @Date: 2023-08-24 14:09:00
 * @LastEditTime: 2023-08-26 08:34:22
 * @LastEditors: yanyan-li yanyan.li.camp@gmail.com
 * @Description:
 * @FilePath: /JointFactorGraph/examples/test_line_rect_intersection.cc
 */

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

void SamplePoints(Eigen::Vector3d &pts_s, Eigen::Vector3d &pts_e, std::vector<Eigen::Vector3d> &sample_pts);

int main(int argc, char **argv)
{
    // generate endpoints of a 2D line
    Eigen::Vector3d point_s, point_e;
    point_s << -300, 200, 5;
    point_e << 120, 430, 3;
    Eigen::Vector3d direction = (point_s - point_e).normalized();
    double line_distance = (point_s - point_e).norm();

    std::cout << "line: " << direction << "," << line_distance << std::endl;
    std::vector<Eigen::Vector3d> sample_pts;
    SamplePoints(point_s, point_e, sample_pts);

    cv::Mat img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    for (auto pts_i : sample_pts)
        cv::circle(img, cv::Point(pts_i(0), pts_i(1)), 2, cv::Scalar(pts_i(2) * 25.5, 255 - pts_i(2) * 25.5, 125));

    cv::imshow("2D Viewer", img);
    if (cv::waitKey() == 27)
        cv::imwrite("../test_line_pts.png", img);

    return 0;
}

void SamplePoints(Eigen::Vector3d &pts_s, Eigen::Vector3d &pts_e, std::vector<Eigen::Vector3d> &sample_pts)
{

    //.normalized();
    double line_distance = (pts_s - pts_e).norm();

    int point_num = 100;
    Eigen::Vector3d sample_direction = (pts_s - pts_e) / 100;

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d pt_i = pts_e + sample_direction * i;
        if ((pt_i(0) < 0) || (pt_i(0) > 479) || (pt_i(1) < 0) || (pt_i(1) > 639) || (pt_i(2) < 0))
            continue;
        sample_pts.push_back(pt_i);
    }
}
