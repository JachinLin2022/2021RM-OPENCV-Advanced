/**
 * 大三组
 * Examination
 * @author frezcirno
 */

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "detect.hpp"
#include "readParameter.hpp"

using namespace std;
using namespace cv;

int main()
{
    Mat cameraMatrix, distCoeffs;
    if (readParameter(cameraMatrix, distCoeffs) != 0)
    {
        cout << "read parameter failed" << endl;
        return -1;
    }

    VideoCapture cap = VideoCapture(0);
    if (!cap.isOpened())
    {
        cout << "open camera failed" << endl;
        return -1;
    }

    Mat test = imread("test.png");
    detect(test);
    imwrite("test_out.png", test);

    Mat _frame, frame;
    while (true)
    {
        double t = getTickCount();

        if (waitKey(10) == 'q')
        {
            break;
        }

        cap >> _frame;
        undistort(_frame, frame, cameraMatrix, distCoeffs);
        auto armors = detect(frame);

        // getTickcount函数：返回从操作系统启动到当前所经过的毫秒数
        // getTickFrequency函数：返回每秒的计时周期数
        // t为该处代码执行所耗的时间,单位为秒,fps为其倒数
        t = (getTickCount() - t) / getTickFrequency();
        float fps = 1.0 / t;

        putText(frame, "FPS:" + to_string(fps), Point(5, 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
        imshow("cap", frame);
    }

    cap.release();
    destroyAllWindows();

    return 0;
}