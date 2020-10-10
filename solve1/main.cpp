#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include <ctime>
#include <functional>
#include <vector>
#include <cmath>

//定义了一个矩形列表，每一维度定义了一个矩形，矩形的每一维度是顺时针方向的四个点
typedef std::vector<std::vector<cv::Point2f>> RectAreaList;

/*
@brief 用来显示图片，主要是方便以一定大小进行显示
@param img_name：要创建的窗口名称
@param img：mat形式的图片信息
@param img_size：图片的大小
*/
void showImg(const cv::String &img_name, cv::Mat &img, const cv::Size &img_size)
{
    cv::namedWindow(img_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(img_name, img_size);
    cv::imshow(img_name, img);
}

/*
@brief 用来读取图片
@param file_name：要读取的图片的路径
@return 读取到的图片
*/
cv::Mat readImg(const cv::String &file_name)
{
    cv::Mat img = cv::imread(file_name);
    if (!img.data)
    {
        std::cout << "打开失败，请将图片放于可执行文件同级目录" << std::endl;
        exit(0);
    }
    return img;
}

/*
@brief 由正常图片转换为灰度图片
@param img：原始图片
@return 处理形成的灰度图片
*/
cv::Mat normalToGray(cv::Mat &img)
{
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, CV_BGR2GRAY);
    return gray_img;
}

/*
@brief 由正常图片转换为hsv图片
@param img：原始图片
@return hsv图片
*/
cv::Mat normalToHsv(cv::Mat &img)
{
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, CV_BGR2HSV);
    return hsv_img;
}

/*
@brief 由灰度图片转换为二值化图片
@param gray_img：灰度图片
@param 阈值
@return 二值化图片
*/
cv::Mat grayToBin(cv::Mat &gray_img, const int threshold_value)
{
    cv::Mat bin_conver_img;
    cv::threshold(gray_img, bin_conver_img, threshold_value, 255, CV_THRESH_BINARY);
    return bin_conver_img;
}

/*
@brief 由hsv转换为二值化图片
@param hsv_img：hsv图片
@param num：长度为6的数组，依次表示三个通道的最小值与三个通道的最大值
@return 二值化图片
*/
cv::Mat hsvToBin(cv::Mat &hsv_img, const int *num)
{
    cv::Mat bin_img;
    cv::inRange(hsv_img, cv::Scalar(num[0], num[1], num[2]), cv::Scalar(num[3], num[4], num[5]), bin_img);
    return bin_img;
}

/*
@brief 对二值化图片处理，找到轮廓，返回的每一维是四个坐标，表示一个方框
@param bin_img：输入的二值化图片
*/
RectAreaList getRectArea(cv::Mat &bin_img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //注意这里可以进行调参
    cv::findContours(bin_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> rect_list;
    cv::Point2f point_list[4];
    RectAreaList ret;
    for (auto &it : contours)
    {
        std::vector<cv::Point2f> point_ve;
        //获得矩形框的四个顶点
        cv::minAreaRect(it).points(point_list);
        for (int i = 0; i < 4; i++)
        {
            point_ve.push_back(point_list[i]);
        }
        ret.push_back(point_ve);
    }
    return ret;
}

/*
@brief 根据条件选择出新的矩形框
@param getRectArea：原始的矩形框集合
@param select：传入的选择函数
@return 被选择的矩形框集合
*/
RectAreaList selectRectArea(RectAreaList &getRectArea, const std::function<bool(std::vector<cv::Point2f> &)> &select)
{
    RectAreaList ret;
    for (auto &it : getRectArea)
    {
        //通过传入的函数进行选择
        if (select(it))
        {
            ret.push_back(it);
        }
    }
    return ret;
}

/*
@brief 计算两个点之间的距离
*/
double pointDis(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/*
@brief 计算一个矩形框的中心位置
@param 一个具有四个点的向量
@return 中心点
*/
cv::Point2f rectCenter(const std::vector<cv::Point2f> &r)
{
    cv::Point2f center_point;
    double x_all = 0;
    double y_all = 0;
    //计算四个点的平均值
    for (int i = 0; i < 4; i++)
    {
        x_all += r[i].x;
        y_all += r[i].y;
    }
    center_point.x = x_all / 4;
    center_point.y = y_all / 4;
    return center_point;
}

/*
@brief 返回两个点的中心点
@param 两个点
@return 中心点
*/
cv::Point2f midPoint(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}

/*
@brief 对两个矩形框进行排序，按照y的值从小到大，x的值从小到大排序，指定了一个y的区间，在一定误差内的认为是同一高度
*/
bool rectCmp(const std::vector<cv::Point2f> &r1, const std::vector<cv::Point2f> &r2)
{
    //y的波动范围阈值
    constexpr double y_eps = 200;
    cv::Point2f r1_center = rectCenter(r1);
    cv::Point2f r2_center = rectCenter(r2);
    if (fabs(r1_center.y - r2_center.y) < y_eps)
    {
        return r1_center.x < r2_center.x;
    }
    return r1_center.y < r2_center.y;
}

/*
@brief 判断一个矩形框是否能入选，目前是根据长宽比、面积、矩形的横竖情况进行选择
*/
bool selectMethod(std::vector<cv::Point2f> &get_list)
{
    constexpr double div_high = 15;         //长宽比上界
    constexpr double div_low = 3.1;         //长宽比下界
    constexpr double square_low = 800;      //面积下界
    const double sin_low = sqrt(2) / 2; //倾斜的sin下界
    //获得长边和短边
    double max_value = std::max(pointDis(get_list[0], get_list[1]), pointDis(get_list[1], get_list[2]));
    double min_value = std::min(pointDis(get_list[0], get_list[1]), pointDis(get_list[1], get_list[2]));

    double div_value = max_value / min_value;
    //长宽比淘汰
    if (div_value > div_high || div_value < div_low)
    {
        return false;
    }

    //面积淘汰
    if (max_value * min_value < square_low)
    {
        return false;
    }
    //倾斜程度淘汰
    //计算长边的x的差值，便于和长边计算sin值
    double x_dis;
    if (pointDis(get_list[0], get_list[1]) > pointDis(get_list[0], get_list[3]))
    {
        x_dis = fabs(get_list[0].x - get_list[1].x);
    }
    else
    {
        x_dis = fabs(get_list[0].x - get_list[3].x);
    }
    if (x_dis / max_value > sin_low)
    {
        return false;
    }

    return true;
}

/*
@brief 在一个图像上绘制矩形框
@param img：图片
@param rect_list：矩形框列表
@param color：scalar形式的三通道颜色
@param thickness：粗细
*/
void drawRectInImg(cv::Mat &img, RectAreaList &rect_list, const cv::Scalar &color, const int thickness)
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
@brief 根据二值化图像判断某一个点的周围一小部分是否为白色
@param img：图片
@param p：点
*/
bool judgeWhite(cv::Mat &img, const cv::Point2f &p)
{
    //检测范围为2*range的正方形区域
    constexpr int range = 10;
    //白色方框的比例阈值
    constexpr double min_limit = 0.8;
    int x = static_cast<int>(p.x);
    int y = static_cast<int>(p.y);
    int cnt = 0;
    for (int i = -range; i < range; i++)
    {
        for (int j = -range; j < range; j++)
        {
            //只检测了单通道
            if (img.at<uchar>(y + i, x + j) == 255)
            {
                cnt++;
            }
        }
    }
    if (static_cast<double>(cnt) / range * range >= min_limit)
    {
        return true;
    }
    return false;
}

/*
@brief 对排好序的矩阵列表进行匹配
@param bin_img：根据数字二值化以后的图片，便于匹配
@param rect_list：排好序的矩阵序列
@return 一定是偶数数量的矩阵，从头开始，相邻的两两配对
*/
RectAreaList matchRect(cv::Mat &bin_img, const RectAreaList &rect_list)
{
    size_t i = 0;
    RectAreaList match_rect;
    while (i < rect_list.size())
    {
        if (i == rect_list.size() - 1)
        {
            break;
        }
        auto p1 = rectCenter(rect_list[i]);
        auto p2 = rectCenter(rect_list[i + 1]);
        //因为是排好序的，所以如果p1.x大的话说明是换行的情况，如果是相邻的话则检测中心区域
        /*
        1 2 3 4
        5 6 7 8
        例如对于以上8个排好序并且标好号的灯条来说，依次遍历，首先尝试配对1号2号
        发现2号x大，继续检测1号2号之间是否可能为数字，如果是，则下次匹配3号4号
        否则，匹配2号3号
        当假如配对到4号5号时，则会出现5号的x小，这种情况一定不存在，因为灯条已经按照y和x的顺序排好了序，所以这种不予匹配
        */
        if (p1.x < p2.x && judgeWhite(bin_img, midPoint(p1, p2)))
        {
            match_rect.push_back(rect_list[i]);
            match_rect.push_back(rect_list[i + 1]);
            i += 2;
        }
        else
        {
            i++;
        }
    }
    return match_rect;
}

/*
@brief 根据匹配的矩形列表找到装甲板框，匹配方法是以两个灯条的中心点，根据fac竖直划两条线，之后连成一个矩形
@param match_list：匹配的矩形列表
@return 装甲板的矩形列表
*/
RectAreaList getPlateList(const RectAreaList &match_list)
{
    //左右两边化竖线的长度扩大比例，线的长度为2*fac*灯条长度
    constexpr double fac = 1.35;
    RectAreaList plate_list;
    for (size_t i = 0; i < match_list.size(); i += 2)
    {
        std::vector<cv::Point2f> tmp_ve;
        double p1_max_value = fac * std::max(pointDis(match_list[i][0], match_list[i][1]), pointDis(match_list[i][1], match_list[i][2]));
        double p2_max_value = fac * std::max(pointDis(match_list[i + 1][0], match_list[i + 1][1]), pointDis(match_list[i + 1][1], match_list[i + 1][2]));
        auto p1 = rectCenter(match_list[i]);
        auto p2 = rectCenter(match_list[i + 1]);
        tmp_ve.push_back(cv::Point2f(p1.x, p1.y - p1_max_value));
        tmp_ve.push_back(cv::Point2f(p2.x, p2.y - p2_max_value));
        tmp_ve.push_back(cv::Point2f(p2.x, p2.y + p2_max_value));
        tmp_ve.push_back(cv::Point2f(p1.x, p1.y + p1_max_value));
        plate_list.push_back(tmp_ve);
    }
    return plate_list;
}

/*
@brief 单纯的在一个黑幕上绘制矩形框，为了展示方便
@param name：显示窗口的名称
@param gray_img：灰度图，为了生成黑幕图片
@param rect_list：矩形框列表
@param size：窗口尺寸
*/
void showDemo(const cv::String name, cv::Mat &gray_img, RectAreaList &rect_list, const cv::Size &size)
{
    cv::Mat &&demo = grayToBin(gray_img, 255);
    drawRectInImg(demo, rect_list, cv::Scalar(255, 255, 255), 2);
    showImg(name, demo, size);
}

int main()
{
    //图片路径
    const cv::String img_get = "test.jpg";
    //窗口大小，根据给定图片设置了3：2大小
    constexpr int img_width = 900;
    const cv::Size img_size(img_width, img_width / 3 * 2);
    //灯条二值化阈值,tv：threshold_value
    constexpr int tv_light = 215;
    //数字二值化阈值
    constexpr int tv_num = 155;
    //画框的颜色和粗细
    const auto rect_color = cv::Scalar(242, 250, 247);
    const int rect_thickness = 10;
    //输出路径
    const cv::String img_out = "armor.jpg";

    //读取图片并且进行判断,返回右值，减少拷贝
    cv::Mat &&img = readImg(img_get);

    //获取灰度图片
    cv::Mat &&gray_img = normalToGray(img);
    //获取灯条二值化图片
    cv::Mat &&bin_img_light = grayToBin(gray_img, tv_light);
    showImg("灯条二值化图片", bin_img_light, img_size);
    //获取数字二值化图片
    cv::Mat &&bin_img_num = grayToBin(gray_img, tv_num);
    showImg("数字二值化图片", bin_img_num, img_size);

    //获取灯条轮廓
    RectAreaList &&rect_area = getRectArea(bin_img_light);
    showDemo("灯条轮廓", gray_img, rect_area, img_size);

    //筛选轮廓
    RectAreaList &&selected_rect_area = selectRectArea(rect_area, selectMethod);
    showDemo("筛选轮廓", gray_img, selected_rect_area, img_size);
    //对轮廓进行排序
    std::sort(selected_rect_area.begin(), selected_rect_area.end(), rectCmp);
    //获得配对以后的矩形框
    RectAreaList &&match_rect_list = matchRect(bin_img_num, selected_rect_area);
    showDemo("灯条与配对结果", gray_img, match_rect_list, img_size);
    //获得装甲板矩形框
    RectAreaList &&plate_list = getPlateList(match_rect_list);
    showDemo("框选装甲板", gray_img, plate_list, img_size);
    //在原图片上绘制矩形框
    drawRectInImg(img, selected_rect_area, rect_color, rect_thickness);
    drawRectInImg(img, plate_list, rect_color, rect_thickness);
    showImg("result", img, img_size);
    cv::imwrite(img_out, img);

    cv::waitKey();

    return 0;
}