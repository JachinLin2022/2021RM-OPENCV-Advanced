#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/time.h>

cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));        // 摄像机内参数矩阵
cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));          // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
float board_height = 60;  // 装甲板的高度，单位：毫米
float board_width = 135;  // 装甲板的宽度，单位：毫米

/*************************************************
Board类： 装甲板类，包含了识别出的各个装甲板的信息
相关变量：
left_top: 左上角点的坐标
left_down: 左下角点的坐标
right_top: 右上角点的坐标
right_down: 右下角点的坐标
distance: 装甲板与摄像头中心的距离
yaw: 装甲板的yaw角
pitch: 装甲板的pitch角
*************************************************/
struct Board {
    cv::Point2f left_top;
    cv::Point2f left_down;
    cv::Point2f right_top;
    cv::Point2f right_down;
    float distance;
    float yaw;
    float pitch;
};

/****************************************
函数名：CallBack_Demo
功能：
    回调函数，无操作
****************************************/
void CallBack_Demo(int, void*)
{
    return;
}

/****************************************
函数名：getCurrentTime
功能：
    取得当前程序运行时间
****************************************/
long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


/****************************************
函数名：get_hsv_range
功能：
    对于读入的图片，动态调整HSV的阈值范围
    以提取目标物体
****************************************/
void get_hsv_range(cv::InputArray src, std::vector<int>& hsv_range)
{
    // 250 255
    // 110 160 0 115 151 255
    // 162 181 70 255 27 255
    int h_min = 0, h_max = 255, s_min = 0, s_max = 63, v_min = 240, v_max = 255;        // 设置初始值

    /* 创建窗口 */
    cv::namedWindow("TrackBars");
    cv::resizeWindow("TrackBars", 640, 240);

    /* 创建滑动栏 */
    cv::createTrackbar("Hue Min", "TrackBars", &h_min, 255, CallBack_Demo);
    cv::createTrackbar("Hue Max", "TrackBars", &h_max, 255, CallBack_Demo);
    cv::createTrackbar("Sat Min", "TrackBars", &s_min, 255, CallBack_Demo);
    cv::createTrackbar("Sat Max", "TrackBars", &s_max, 255, CallBack_Demo);
    cv::createTrackbar("Val Min", "TrackBars", &v_min, 255, CallBack_Demo);
    cv::createTrackbar("Val Max", "TrackBars", &v_max, 255, CallBack_Demo);

    while (true) {
        cv::Mat imgHSV, mask;
        /* 转HSV格式图片 */
        cvtColor(src, imgHSV, cv::COLOR_BGR2HSV);

        /* 从滑动栏获取当前相关值 */
        h_min = cv::getTrackbarPos("Hue Min", "TrackBars");
        h_max = cv::getTrackbarPos("Hue Max", "TrackBars");
        s_min = cv::getTrackbarPos("Sat Min", "TrackBars");
        s_max = cv::getTrackbarPos("Sat Max", "TrackBars");
        v_min = cv::getTrackbarPos("Val Min", "TrackBars");
        v_max = cv::getTrackbarPos("Val Max", "TrackBars");

        /* 设置阈值，二值化处理，得到掩模mask */
        cv::inRange(imgHSV, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);

        /* 显示原图片、HSV格式图片、Mask掩模图片 */
        cv::namedWindow("Origin", 0);
        cv::imshow("Origin", src);

        cv::namedWindow("HSV", 0);
        cv::imshow("HSV", imgHSV);

        cv::namedWindow("Mask", 0);
        cv::imshow("Mask", mask);

        if (cv::waitKey(30) >= 0) {
            break;
        }

    }

    /* 回传HSV阈值 */
    hsv_range.push_back(h_min);
    hsv_range.push_back(h_max);
    hsv_range.push_back(s_min);
    hsv_range.push_back(s_max);
    hsv_range.push_back(v_min);
    hsv_range.push_back(v_max);
}

/*********************************************************************
函数功能：测量装甲板与摄像头之间的各项参数，包括距离、yaw角、pitch角
输入参数:
imagePoints: 图像上装甲板各点的坐标，按照左上、左下、右下、右上的顺序
cameraMatrix: 摄像头参数矩阵
distCoeffs: 摄像头畸变矩阵
board_height: 装甲板的高度，单位: 毫米
board_width: 装甲板的宽度，单位: 毫米
输出参数:
pitch: 装甲板的pitch角
yaw: 装甲板的yaw角
distance: 装甲板与摄像头中心的距离
*********************************************************************/
void measure_distance(std::vector<cv::Point2f> imagePoints, cv::InputArray cameraMatrix, cv::InputArray distCoeffs, float board_height, float board_width, float& pitch, float& yaw, float& distance)
{
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_32FC1);  // 旋转矩阵，装甲板的各旋转角
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_32FC1);  // 位移矩阵，装甲板与摄像头的位移
    std::vector<cv::Point3f> objectPoints;
    /* 下列四行代码根据imagePoints的点的顺序自定义世界坐标系下装甲板各点的坐标 */
    /* 以装甲板的中心作为世界坐标系的坐标原点，并按照实际的装甲板长宽进行设定坐标 */
    objectPoints.push_back(cv::Point3f(-board_width / 2, board_height / 2, 0));
    objectPoints.push_back(cv::Point3f(-board_width / 2, -board_height / 2, 0));
    objectPoints.push_back(cv::Point3f(board_width / 2, -board_height / 2, 0));
    objectPoints.push_back(cv::Point3f(board_width / 2, board_height / 2, 0));

    /* 使用solvePnP函数得到摄像头的位移矩阵和旋转矩阵 */
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);

    /* 从得到的位移矩阵和旋转矩阵中取出所需数据 */
    pitch = rVec.at<double>(0, 0);
    yaw = rVec.at<double>(2, 0);
    double dis_x = tVec.at<double>(0, 0);
    double dis_y = tVec.at<double>(1, 0);
    double dis_z = tVec.at<double>(2, 0);
    distance = sqrt(dis_x * dis_x + dis_y * dis_y + dis_z * dis_z);
}

/****************************************
函数名：Draw_a_Board_Rect
功能：
    绘制一个矩形，标注装甲板
****************************************/
void Draw_a_Board_Rect(cv::InputOutputArray img, Board board_info, int thickness = 1)
{
    cv::line(img, board_info.left_top, board_info.right_top, cv::Scalar(255, 0, 255), 2);
    cv::line(img, board_info.right_top, board_info.right_down, cv::Scalar(255, 0, 255), 2);
    cv::line(img, board_info.right_down, board_info.left_down, cv::Scalar(255, 0, 255), 2);
    cv::line(img, board_info.left_down, board_info.left_top, cv::Scalar(255, 0, 255), 2);
}

/****************************************
函数名：Draw_Boards_Rects
功能：
    绘制一个矩形，标注装甲板
****************************************/
void Draw_Boards_Rects(cv::InputOutputArray img, std::vector<Board> board_info)
{
    /* 绘制所有识别的装甲板 */
    for (int i = 0; i < board_info.size(); ++i)
        Draw_a_Board_Rect(img, board_info[i], 2);
}

/****************************************
函数名：Draw_Text
功能：
    在图像上显示文本信息
****************************************/
void Draw_Text(cv::InputOutputArray img, std::vector<std::string> text)
{
    int lines_num = text.size();

    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1;
    int thickness = 1;
    int baseline;
    int text_length_max;		// 记录最大字符串长度
    cv::Size text_size;
    cv::Point origin;		// 记录文本框左上角坐标
    cv::Point current;
    Board board;


    /* 获取文本长宽 */
    if (lines_num > 0) {
        text_size = cv::getTextSize(text[0], font_face, font_scale, thickness, &baseline);
        text_length_max = text[0].length();
    }

    /* 获取最大字符串长度 */
    for (int i = 0; i < lines_num; ++i)
        if (text_length_max < text[i].length())
            text_length_max = text[i].length();
        else
            continue;

    /* 重设置文本框宽度，以最大字符串长度为准 */
    text_size.width = text_length_max * text_size.width / text[0].length();

    /* 计算左上角点坐标 */
    //origin.x = img.cols() / 2 - text_size.width / 2;
    //origin.y = img.rows() / 2 - text_size.height * lines_num / 2;
    origin.x = 10;
    origin.y = 40;

    for (int i = 0; i < lines_num; ++i) {
        current.x = origin.x;
        current.y = origin.y + text_size.height * i;

        cv::putText(img, text[i], current, font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 8, 0);
    }

    int extern_board = 5;		// 设置文本框扩展宽度
    origin.y = origin.y - text_size.height;		// 文本框左上角坐标减去文本条高度

    /* 计算文本框的四个点的坐标 */
    board.left_top = cv::Point2f(origin.x - extern_board, origin.y - extern_board);
    board.left_down = cv::Point2f(origin.x - extern_board, origin.y + extern_board + lines_num * text_size.height);
    board.right_top = cv::Point2f(origin.x + extern_board + text_size.width, origin.y - extern_board);
    board.right_down = cv::Point2f(origin.x + extern_board + text_size.width, origin.y + extern_board + lines_num * text_size.height);

    /* 绘制文本框 */
    Draw_a_Board_Rect(img, board);

}

/****************************************
函数名：Draw_Info
功能：
    显示提示信息，FPS、灯条数、装甲板数等
****************************************/
void Draw_Info(cv::InputOutputArray img, int fps, int light_num, std::vector<Board> boards)
{
    std::vector<std::string> text;

    /* 取装甲板个数 */
    int board_size = boards.size();

    std::string text_fps("FPS:");
    std::string text_light("lights number:");
    std::string text_board_num("boards number:");

    /* 设置显示信息，FPS、灯条数、装甲板数 */
    text_fps += std::to_string(fps);
    text.push_back(text_fps);
    text_light += std::to_string(light_num);
    text.push_back(text_light);
    text_board_num += std::to_string(board_size);
    text.push_back(text_board_num);

    /* 设置显示信息，距离 */
    for (int i = 0; i < board_size; ++i) {
        std::string text_board_distance;
        text_board_distance += "No." + std::to_string(i + 1) + "board distance: ";
        text_board_distance += std::to_string(boards[i].distance);
        text.push_back(text_board_distance);
    }

    /* 显示文本 */
    Draw_Text(img, text);
}

/****************************************
函数名：get_lights_mask
功能：
    获取图像掩模
****************************************/
void get_lights_mask(cv::InputArray img, cv::OutputArray mask)
{
    long start, end;
    cv::Mat imgHSV, imgBlur, maskBlur;
    //6 112 0 163 215 255
    // 0 60 11 40 230 255
    /* 设置阈值 */
    int h_min = 0, h_max = 255, s_min = 0, s_max = 63, v_min = 240, v_max = 255;

    start = getCurrentTime();
    /* 转HSV格式 */
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);
    end = getCurrentTime();
    std::cout << "转HSV格式：" << end - start << std::endl;

    start = getCurrentTime();
    /* 以指定范围的阈值进行二值化 */
    cv::inRange(imgHSV, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), maskBlur);
    end = getCurrentTime();
    std::cout << "以指定范围的阈值进行二值化：" << end - start << std::endl;

    start = getCurrentTime();
    /* 闭操作填充黑点 */
    //cv::morphologyEx(maskBlur, mask, cv::MORPH_OPEN, cv::Mat(3, 3, CV_8U));
    cv::morphologyEx(maskBlur, mask, cv::MORPH_CLOSE, cv::Mat(5, 5, CV_8U));
    end = getCurrentTime();
    std::cout << "闭操作填充黑点：" << end - start << std::endl;

}

/****************************************
函数名：get_lights_contours
功能：
    提取掩模（二值化图像）轮廓
****************************************/
void get_lights_contours(cv::InputOutputArray mask, std::vector<std::vector<cv::Point>>& contours)
{
    /* 提取轮廓 */
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
}

/****************************************
函数名：get_lights_rects
功能：
    根据提取到的轮廓，提取旋转矩形
****************************************/
void get_lights_rects(std::vector<std::vector<cv::Point>> contours, std::vector<cv::RotatedRect>& rects)
{
    cv::Mat imgContours(600, 800, CV_64FC1);
    for (int i = 0; i < contours.size(); i++) {

        /* 计算区域大小 */
        double area = contourArea(contours[i], true);

        /* 过滤一些区域 */
        if (abs(area) > 5.0) {

            /* 将轮廓显示 */
            cv::drawContours(imgContours, contours, i, cv::Scalar(255, 0, 0));

            /* 提取最小旋转矩形 */
            cv::RotatedRect r = cv::minAreaRect(contours[i]);

            /* 获取该矩形四个点的坐标 */
            cv::Point2f vertices[4];
            r.points(vertices);

            /* 计算宽高 */
            float w, h;
            h = sqrt(powf((vertices[0].x - vertices[1].x), 2) + powf((vertices[0].y - vertices[1].y), 2));
            w = sqrt(powf((vertices[1].x - vertices[2].x), 2) + powf((vertices[1].y - vertices[2].y), 2));

            /* 设置挑选条件 */
            if ((h / w >= 2.0 || w / h >= 2.0) && (w * h > 0.0)) {
                /* 添加 */
                rects.push_back(r);
            }
        }
    }
    /* 显示轮廓信息 */
    cv::namedWindow("imgContours", 0);
    cv::imshow("imgContours", imgContours);
}


/*************************************************************************
函数功能: 找到灯条矩形的短边，并算出灯条上下两边的中心点作为灯条的上下两点
输入参数:
rect: 灯条矩形
输出参数:
p1: 灯条的上方中点
p2: 灯条的下方中点
*************************************************************************/
void find_short_side(cv::RotatedRect rect, cv::Point& p1, cv::Point& p2)
{
    int pos;
    double min = 100000000.0;

    /* 提取出矩形的四个点 */
    cv::Point2f points[4];
    rect.points(points);

    /* 找到矩形所有边中的最短边 */
    for (int i = 0; i < 4; i++) {
        double tmp = (points[i].x - points[(i + 1) % 4].x) * (points[i].x - points[(i + 1) % 4].x) + (points[i].y - points[(i + 1) % 4].y) * (points[i].y - points[(i + 1) % 4].y);
        if (tmp < min) {
            min = tmp;
            pos = i;
        }
    }

    /* 计算得到灯条的上下两边的中点 */
    int x1, x2, y1, y2;
    x1 = (points[pos].x + points[(pos + 1) % 4].x) / 2;
    y1 = (points[pos].y + points[(pos + 1) % 4].y) / 2;
    x2 = (points[(pos + 2) % 4].x + points[(pos + 3) % 4].x) / 2;
    y2 = (points[(pos + 2) % 4].y + points[(pos + 3) % 4].y) / 2;

    /* 根据坐标分别找出上方点和下方点的坐标并保存至p1和p2 */
    if (y1 < y2) {
        p1.x = x1;
        p1.y = y1;
        p2.x = x2;
        p2.y = y2;
    }
    else {
        p1.x = x2;
        p1.y = y2;
        p2.x = x1;
        p2.y = y1;
    }
}

/*****************************************************************
函数功能: 判断两个灯条矩形是否为一对，即是否为同一装甲板的两个灯条
输入参数:
rect1: 灯条1
rect2: 灯条2
*****************************************************************/
bool judging_rect(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    cv::Point points[2][2];  // 表示两个灯条的上下两点，如points[0][0]表示第一个灯条的上点，points[0][1]表示第一个灯条的下点
    /* 找到两个灯条的上下两点 */
    find_short_side(rect1, points[0][0], points[0][1]);
    find_short_side(rect2, points[1][0], points[1][1]);
    double threshold = 0.6;  // 表示两个灯条构成的大矩形的角的cosine值的阈值

    /* 下列代码分别计算出矩形左上角和右下角的cosine值和四边的长度 */
    double cosine1, cosine2, dis_width1, dis_height1, dis_width2, dis_height2;
    dis_width1 = sqrt((points[0][0].x - points[0][1].x) * (points[0][0].x - points[0][1].x) + (points[0][0].y - points[0][1].y) * (points[0][0].y - points[0][1].y));
    dis_height1 = sqrt((points[0][0].x - points[1][0].x) * (points[0][0].x - points[1][0].x) + (points[0][0].y - points[1][0].y) * (points[0][0].y - points[1][0].y));
    dis_height2 = sqrt((points[1][1].x - points[0][1].x) * (points[1][1].x - points[0][1].x) + (points[1][1].y - points[0][1].y) * (points[1][1].y - points[0][1].y));
    dis_width2 = sqrt((points[1][1].x - points[1][0].x) * (points[1][1].x - points[1][0].x) + (points[1][1].y - points[1][0].y) * (points[1][1].y - points[1][0].y));
    int x1 = points[0][1].x - points[0][0].x;
    int y1 = points[0][1].y - points[0][0].y;
    int x2 = points[1][0].x - points[0][0].x;
    int y2 = points[1][0].y - points[0][0].y;
    int x3 = points[0][1].x - points[1][1].x;
    int y3 = points[0][1].y - points[1][1].y;
    int x4 = points[1][0].x - points[1][1].x;
    int y4 = points[1][0].y - points[1][1].y;
    cosine1 = fabs((x1 * x2 + y1 * y2) / (dis_width1 * dis_height1));
    cosine2 = fabs((x3 * x4 + y3 * y4) / (dis_width2 * dis_height2));

    /* 根据设定的阈值判断两个灯条组成的是否是装甲板 */
    if (cosine1 < threshold && cosine2 < threshold && dis_height1 / dis_width1 > 1.5 && dis_height1 / dis_width1 < 2.8 && dis_height2 / dis_width2 > 1.5 && dis_height2 / dis_width2 < 2.8)
        return true;
    else
        return false;
}


/*********************************************************************************
函数功能: 根据找到的所有灯条，找出他们之间的所有配对，即找出每个装甲板所对应的灯条
函数输入: 
r_rects: 所有的灯条矩阵
函数输出:
matched_pairs: 所有配对的灯条在r_rects中的index
*********************************************************************************/
void matching_lights(const std::vector<cv::RotatedRect>& r_rects, std::vector<std::vector<int>>& matched_pairs)
{
    double angle_threshold = 5.0;
    for (int i = 0; i<int(r_rects.size()); i++) {
        std::vector<int> tmp_record;
        for (int j = 0; j<int(r_rects.size()); j++) {
            if (j == i)
                continue;
            std::vector<int> tmp;
            tmp.push_back(j);
            tmp.push_back(i);
            bool flag = false;
            for (int k = 0; k<int(matched_pairs.size()); k++) {
                if (matched_pairs[k][0] == tmp[0] && matched_pairs[k][1] == tmp[1]) {
                    flag = true;
                    break;
                }
            }
            if (flag)
                continue;
            if (judging_rect(r_rects[i], r_rects[j]))
                tmp_record.push_back(j);
        }
        int pos = 0;
        double min = 655360000;
        for (int j = 0; j<int(tmp_record.size()); j++) {
            double dis = (r_rects[i].center.x - r_rects[tmp_record[j]].center.x) * (r_rects[i].center.x - r_rects[tmp_record[j]].center.x) + (r_rects[i].center.x - r_rects[tmp_record[j]].center.x) * (r_rects[i].center.y - r_rects[tmp_record[j]].center.y);
            if (dis < min) {
                pos = tmp_record[j];
                min = dis;
            }
        }
        if (int(tmp_record.size()) > 0) {
            std::vector<int> tmp;
            tmp.push_back(i);
            tmp.push_back(pos);
            matched_pairs.push_back(tmp);
        }
    }
}


/*******************************************
函数功能: 根据输入的图片找到所有装甲板的位置
输入参数:
img: 输入图片
输出参数:
boards: 所找到的装甲板类
lights_num: 所找到的灯条数目
*******************************************/
void find_lights_pos(cv::Mat img, std::vector<Board>& boards, int& lights_num)
{
    long start, end;
    cv::Mat mask;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::RotatedRect> rects;
    std::vector<int> hsv_range;

    // 获取灯条二值化掩膜
    get_lights_mask(img, mask);

    start = getCurrentTime();
    // 获取掩膜轮廓
    get_lights_contours(mask, contours);
    end = getCurrentTime();
    std::cout << "获取掩膜轮廓：" << end - start << std::endl;

    cv::namedWindow("Mask", 0);
    cv::resizeWindow("Mask", cv::Size(800, 600));
    cv::imshow("Mask", mask);

    start = getCurrentTime();
    // 获取灯条矩形
    get_lights_rects(contours, rects);
    end = getCurrentTime();
    std::cout << "获取灯条矩形：" << end - start << std::endl;
    lights_num = int(contours.size());

    start = getCurrentTime();
    std::vector<std::vector<int>> matched_pairs;
    std::vector<cv::Point> centers;

    /* 找出配对的灯条 */
    matching_lights(rects, matched_pairs);

    /* 找出配对的灯条所对应的装甲板的四个角的坐标 */
    for (int i = 0; i<int(matched_pairs.size()); i++) {
        Board tmp_board;
        cv::Point points[2][2];
        find_short_side(rects[matched_pairs[i][0]], points[0][0], points[0][1]);
        find_short_side(rects[matched_pairs[i][1]], points[1][0], points[1][1]);
        if (points[0][0].x < points[1][0].x) {
            tmp_board.left_top = points[0][0];
            tmp_board.left_down = points[0][1];
            tmp_board.right_top = points[1][0];
            tmp_board.right_down = points[1][1];
        }
        else {
            tmp_board.left_top = points[1][0];
            tmp_board.left_down = points[1][1];
            tmp_board.right_top = points[0][0];
            tmp_board.right_down = points[0][1];
        }
        boards.push_back(tmp_board);
    }
    end = getCurrentTime();
    std::cout << "灯条配对：" << end - start << std::endl;
}


/***************************************
函数功能: 初始化摄像头参数矩阵和畸变矩阵
***************************************/
void init_camera_para()
{
    cameraMatrix = (cv::Mat_ <float>(3, 3) <<
        481.3302276807188, 0, 321.3257883834382,
        0, 479.7663420013469, 221.4541682854603,
        0, 0, 1);
    distCoeffs = (cv::Mat_ <float>(1, 5) <<
        -0.399664946819652, 0.1546067670502199, 0.003442420851164169, -0.0004838476700251068, 0);
}

/***********************************************************
函数功能: 主循环函数，通过摄像头读取到的图像找出其中的装甲板
***********************************************************/
void auto_aiming()
{
    int fps;

    /* 初始化摄像头所需参数 */
    init_camera_para();
    cv::VideoCapture capture;			//打开摄像头
    capture.open(0);
    if (!capture.isOpened())
    {
        printf("[%s][%d]could not load video data...\n", __FUNCTION__, __LINE__);
        return;
    }

    double exposure = capture.get(cv::CAP_PROP_EXPOSURE);
    std::cout << exposure << std::endl;
    capture.set(cv::CAP_PROP_EXPOSURE, 500);

    int i = 2;
    /* 主循环 */
    while (1) {
        cv::Mat img_per_frame, src;
        std::vector<Board> boards;
        long start_time, end_time, start, end;
        start_time = getCurrentTime();
        int lights_num;
        /* 通过摄像头得到每一帧的图片 */
        capture >> img_per_frame;

        src = img_per_frame.clone();

        //        std::vector<int> hsv_range;
        //        get_hsv_range(img_per_frame, hsv_range);

        /* 找到图片中的灯条并返回每个装甲板的中心位置 */
        find_lights_pos(img_per_frame, boards, lights_num);

        /* 计算出每个装甲板距离摄像头的距离、yaw、pitch并输出，以及输出帧数 */
        for (int i = 0; i<int(boards.size()); i++) {
            std::vector<cv::Point2f> imagePoints;
            imagePoints.push_back(boards[i].left_top);
            imagePoints.push_back(boards[i].left_down);
            imagePoints.push_back(boards[i].right_down);
            imagePoints.push_back(boards[i].right_top);
            measure_distance(imagePoints, cameraMatrix, distCoeffs, board_height, board_width, boards[i].pitch, boards[i].yaw, boards[i].distance);
        }
        end_time = getCurrentTime();
        std::cout << "每帧花费时间" << end_time - start_time << std::endl;
        fps = 1000 / int(end_time - start_time);
        std::cout << "fps: " << fps << std::endl;
        Draw_Boards_Rects(img_per_frame, boards);
        Draw_Info(img_per_frame, fps, lights_num, boards);

        /* 输出所得的装甲板 */
        cv::namedWindow("image", 0);
        cv::resizeWindow("image", cv::Size(800, 600));
        cv::imshow("image", img_per_frame);
        cv::waitKey(30);
    }

}

/****************************************
函数名：cheese_pictures
功能：
    拍摄指定数量的照片
****************************************/
void cheese_pictures(const int& count)
{
    cv::VideoCapture capture;
    capture.open(0);		// 打开摄像机
    if (!capture.isOpened()) {
        printf("[%s][%d]could not load video data...\n", __FUNCTION__, __LINE__);
        return;
    }

    /* 拍摄 */
    for (int i = 0; i < count; ++i) {
        /* 指定图片名称 */
        cv::Mat src;
        std::string filename("newboard");
        filename = filename + std::to_string(i + 1) + ".jpg";

        while (true) {
            capture >> src;

            cv::imshow("src", src);

            /* 按任意键进行拍摄 */
            if (cv::waitKey(30) >= 0)
                break;
        }

        /* 存储拍摄照片 */
        std::cout << "第" << i + 1 << "张照片" << std::endl;
        cv::imwrite(filename, src);		// 存储图片
    }
}
int main(int argc, char** argv)
{
    auto_aiming();
    //chess_pictures(100);
    return 0;
}