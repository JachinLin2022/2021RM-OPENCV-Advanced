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
#include "solve.hpp"

using namespace std;
using namespace cv;

int main()
{
    Mat cameraMatrix, distCoeffs;
    if (readParameter(cameraMatrix, distCoeffs) != 0) {
        cout << "read parameter failed" << endl;
        return -1;
    }

    VideoCapture cap = VideoCapture(0);

    // cap.set(CAP_PROP_FRAME_WIDTH, 640);  //宽度
    // cap.set(CAP_PROP_FRAME_HEIGHT, 360);  //高度
    // cap.set(CAP_PROP_FPS, 60);        //帧率 帧/秒
    // cap.set(CAP_PROP_BRIGHTNESS, 50);  //亮度
    // cap.set(CAP_PROP_CONTRAST, 0);       //对比度
    // cap.set(CAP_PROP_SATURATION, 50);     //饱和度
    // cap.set(CAP_PROP_HUE, 50);            //色调
    // cap.set(CAP_PROP_EXPOSURE, 5000);       //曝光

    cout << "CAP_PROP_FRAME_WIDTH = " << cap.get(CAP_PROP_FRAME_WIDTH) << endl;  //宽度
    cout << "CAP_PROP_FRAME_HEIGHT = " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;  //高度
    cout << "CAP_PROP_FPS = " << cap.get(CAP_PROP_FPS) << endl;  //帧率 帧/秒
    cout << "CAP_PROP_BRIGHTNESS = " << cap.get(CAP_PROP_BRIGHTNESS) << endl;  //亮度
    cout << "CAP_PROP_CONTRAST = " << cap.get(CAP_PROP_CONTRAST) << endl;  //对比度
    cout << "CAP_PROP_SATURATION = " << cap.get(CAP_PROP_SATURATION) << endl;  //饱和度
    cout << "CAP_PROP_HUE = " << cap.get(CAP_PROP_HUE) << endl;  //色调
    cout << "CAP_PROP_EXPOSURE = " << cap.get(CAP_PROP_EXPOSURE) << endl;  //曝光

    if (!cap.isOpened()) {
        cout << "open camera failed" << endl;
        return -1;
    }

    Solver solver(cameraMatrix, distCoeffs);

    Mat frame;
    while (true) {
        double t = getTickCount();

        // 读取一帧
        cap >> frame;

        // 检查键盘输入
        int key = waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 't') {
            imwrite(to_string(int((long long)t % INT_MAX)) + ".jpg", frame);
        }

        // 进行装甲板检测
        vector<vector<Light>> lights;
        vector<vector<Armor>> armors;
        detect(frame, lights, armors);

        // getTickcount函数：返回从操作系统启动到当前所经过的毫秒数
        // getTickFrequency函数：返回每秒的计时周期数
        // t为该处代码执行所耗的时间,单位为秒,fps为其倒数
        t = (getTickCount() - t) / getTickFrequency();
        float fps = 1.0 / t;

        // 显示FPS
        putText(frame, "FPS:" + to_string(fps), Point(5, 25),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
        putText(frame, "Light count: " + to_string(lights[0].size()+lights[1].size()), Point(5, 50),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
        putText(frame, "Armor count: " + to_string(armors[0].size()+armors[1].size()), Point(5, 75),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));

        for (int i = 0; i < 2; i++) {
            for (auto &&light : lights[i]) {
                light.draw(frame, Scalar(255, 255 * (1 - i), 255 * i), 3);
            }

            for (auto &&armor : armors[i]) {
                double pitch, yaw, dist;
                solver.solve(armor, pitch, yaw, dist);

                auto center = armor.center;
                putText(frame, "pitch:" + to_string(pitch), center+Point2f(0,-25),
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
                putText(frame, "yaw: " + to_string(yaw), center,
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
                putText(frame, "dist: " + to_string(dist), center+Point2f(0,25),
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));

                armor.draw(frame, Scalar(255, 255 * (1 - i), 255 * i), 3);
            }
        }

        imshow("cap", frame);
    }

    cap.release();

    destroyAllWindows();

    return 0;
}
