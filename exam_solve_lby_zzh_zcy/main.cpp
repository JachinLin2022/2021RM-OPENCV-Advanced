/*
你需要将你处理装甲板的程序搬运到process.hpp中，并且进行相应的改写，需要保证下述函数的接口符合要求

该函数基于你原本的main函数改编

1. vector<vector<Point2f>> processImg(Mat &img)
注意，此处的processImg函数是基于自己的main修改而成的，接受一帧Mat图像
返回的应当是形如std::vector<std::vector<cv::Point2f>>的形式的数据
其中，第一维的每一维表示一个矩形，第二维的大小固定应该为4，每一个内容表示一个点，构成矩形
即返回的是一个矩形向量，每个矩形又由四个点组成
另外注意，去掉在原来main函数中的waitkey等内容，但是内部进行其他图像显示的内容仍可保留，以供调试

以下函数需要另外修改或者实现

2. ??? getInformation(Mat& rvec,Mat& tvec)
接收两个向量，旋转向量与平移向量，由solvePnP算法计算的出，需要对其进行解析，返回相应的解析后需要显示的数据
返回值未作强制规定，可以是单独的值，也可以是用struct或vector存储的多组值，配合后续的显示函数统一即可

3. void drawDataInImg(Mat& img,???& info)
为了显示，在图片上绘制相应的平移以及旋转数据，第一个参数为每一帧图片，第二个参数为getInformation得到的解析后的数据
二者的数据类型需要保持统一

目前的process.hpp是基于我之前的项目改编而来，现在需要把自己之前的项目替换进去

*/

#include "process.hpp"

//打开的摄像头的id
constexpr int CAMERA_ID = 0;
//物体坐标矩
const vector<Point3f> OBJECT_POINTS{
    Point3f(-70, -30, 0), //tl
    Point3f(-70, 30, 0),  //bl
    Point3f(70, 30, 0),   //br
    Point3f(70, -30, 0)   //tr
};

//相机的内置矩阵
const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) << 1.3876574061180204e+03, 0., 3.1928716087925250e+02, 0.,
                           1.3903482311912710e+03, 2.4452573598500422e+02, 0., 0., 1.);
//畸变参数
const Mat DIST_COEFFS = (Mat_<double>(5, 1) << -4.0084468157922820e-01, 1.2633217938566998e-01,
                         -1.4690984700414732e-04, -6.5998940831282315e-04, 0.);

//一些宏定义的参数，描述显示数字的颜色和粗细
const auto rect_color = Scalar(242, 250, 247);
constexpr int rect_thickness = 10;

/*
@brief 在一个图像上绘制矩形框
@param img：图片
@param rect_list：矩形框列表
@param color：scalar形式的三通道颜色
@param thickness：粗细
*/
void drawRectInImg(cv::Mat &img, RectAreaList &rect_list, const cv::Scalar &color = rect_color, const int thickness = rect_thickness)
{
    for (auto &it : rect_list)
    {
        for (int i = 0; i < 4; i++)
        {
            cv::line(img, it[i], it[(i + 1) % 4], color, thickness);
        }
    }
}

/*
@brief 对矩形按照对应世界坐标系下的顺序对点进行排序
@param rect，一个矩形，包含四个点
*/
void arrangeRect(vector<Point2f> &rect)
{
    //对矩形的点进行排序
    sort(rect.begin(), rect.end(), [](const Point2f &l, const Point2f &r) { return l.x < r.x; });
    if (rect[0].y > rect[1].y)
    {
        Point2f temp = rect[0];
        rect[0] = rect[1];
        rect[1] = temp;
    }
    if (rect[2].y < rect[3].y)
    {
        Point2f temp = rect[2];
        rect[2] = rect[3];
        rect[3] = temp;
    }
}

void drawSysInImg(Mat &image, string fpsString, int num_plate, int num_light)
{
    putText(image,                               // 图像矩阵
            fpsString,                           // string型文字内容
            cv::Point(5, 20),                    // 文字坐标，以左下角为原点
            cv::FONT_HERSHEY_SIMPLEX,            // 字体类型
            0.5,                                 // 字体大小
            cv::Scalar(0, 255, 255));            // 字体颜色
    putText(image,                               // 图像矩阵
            "num_plate:" + to_string(num_plate), // string型文字内容
            cv::Point(5, 40),                    // 文字坐标，以左下角为原点
            cv::FONT_HERSHEY_SIMPLEX,            // 字体类型
            0.5,                                 // 字体大小
            cv::Scalar(0, 255, 255));            // 字体颜色
    putText(image,                               // 图像矩阵
            "num_light:" + to_string(num_light), // string型文字内容
            cv::Point(5, 60),                    // 文字坐标，以左下角为原点
            cv::FONT_HERSHEY_SIMPLEX,            // 字体类型
            0.5,                                 // 字体大小
            cv::Scalar(0, 255, 255));            // 字体颜色
}

int main()
{

    Mat image;
    VideoCapture capture;
    //旋转向量与平移向量，是需要通过pnp算法获得的参数
    Mat rvec = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec = Mat::zeros(3, 1, CV_64FC1);
    //打开摄像头，注意修改打开的摄像头参数
    capture.open(CAMERA_ID);
    if (!capture.isOpened())
    {
        cout << "No capture" << endl;
    }

    cout << "Capture is opened" << endl;
    //调整曝光
    capture.set(CAP_PROP_EXPOSURE, 10000);
    double fps;
    char string[10]; // 用于存放帧率的字符串
    int num_light;

    double t = 0;
    while (true)
    {
        capture >> image;
        if (image.empty())
            break;
        /*
        注意，此处的processImg函数是基于自己的main修改而成的，接受一帧Mat图像
        返回的应当是形如std::vector<std::vector<cv::Point2f>>的形式的数据
        其中，第一维的每一维表示一个矩形，第二维的大小固定应该为4，每一个内容表示一个点，构成矩形
        即返回的是一个矩形向量，每个矩形又由四个点组成
        另外注意，去掉在原来main函数中的waitkey等内容，但是内部进行其他图像显示的内容仍可保留，以供调试
        */
        t = (double)cv::getTickCount();
        auto rect_list = processImg(image, num_light);
        //获取一个矩形
        for (auto &rect : rect_list)
        {

            //对点进行排序
            arrangeRect(rect);
            solvePnP(OBJECT_POINTS, rect, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec);

            /*
            getInformation接受两个参数，旋转矩阵与平移矩阵，返回的值可自行定义，有多组数据时可以考虑返回struct
            */
            auto info = getInformation(rvec, tvec);

            /*
            此处的info需要与getInformation类型统一
            */
            drawDataInImg(image, info, rect[0]);
        }
        drawRectInImg(image, rect_list, rect_list.size(), num_light);

        //进行图片的显示
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps = 1.0 / t;
        sprintf(string, "%.2f", fps); // 帧率保留两位小数
        std::string fpsString("FPS:");
        fpsString += string; // 在"FPS:"后加入帧率数值字符串
        // 将帧率信息,装甲板等信息写在输出帧上
        drawSysInImg(image, fpsString, rect_list.size(), num_light);
        imshow("Sample", image);
        if (waitKey(10) >= 0)
            break;
    }

    return 0;
}