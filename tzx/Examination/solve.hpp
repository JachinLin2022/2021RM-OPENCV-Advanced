/**
 * 大三组
 * Examination: Detection Module
 * @author frezcirno
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class Solver
{
    const static int half_x = 13, half_y = 12;

    const static vector<Point3f> SMALL_ARMOR_POINTS_3D;

    Mat cameraMatrix, distCoeffs;

  public:
    Solver(const Mat &cameraMatrix, const Mat &distCoeffs)
        : cameraMatrix(cameraMatrix), distCoeffs(distCoeffs)
    {
    }

    void solve(const RotatedRect &rect, double &x_pitch, double &y_yaw, double &distance)
    {
        vector<Point2f> vertices(4, Point2f());
        rect.points(vertices.data());

        Mat _rvec, tVec;
        solvePnP(Solver::SMALL_ARMOR_POINTS_3D, vertices, cameraMatrix,
            distCoeffs, _rvec, tVec, false, SOLVEPNP_EPNP);
        double x_pos = tVec.at<double>(0, 0);
        double y_pos = tVec.at<double>(1, 0);
        double z_pos = tVec.at<double>(2, 0);

        double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
        double tan_yaw = x_pos / z_pos;
        x_pitch = -atan(tan_pitch) * 180 / CV_PI;
        y_yaw = atan(tan_yaw) * 180 / CV_PI;
        distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    }
};

const vector<Point3f> Solver::SMALL_ARMOR_POINTS_3D({
    Point3f(-half_x, half_y, 0),   // tl top left
    Point3f(half_x, half_y, 0),    // tr top right
    Point3f(half_x, -half_y, 0),   // br below right
    Point3f(-half_x, -half_y, 0),  // bl below left
});