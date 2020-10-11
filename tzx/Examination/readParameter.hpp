/**
 * 大三组
 * Examination: Detection Module
 * @author frezcirno
 */

#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int readParameter(Mat &cameraMatrix, Mat &distCoeffs)
{
    //导入相机内参和畸变系数矩阵
    FileStorage file_storage("out_camera_data.xml", FileStorage::READ);
    if (!file_storage.isOpened())
    {
        return -1;
    }

    file_storage["camera_matrix"] >> cameraMatrix;
    // cout << cameraMatrix << endl;
    file_storage["distortion_coefficients"] >> distCoeffs;
    // cout << distCoeffs << endl;
    file_storage.release();
    return 0;
}