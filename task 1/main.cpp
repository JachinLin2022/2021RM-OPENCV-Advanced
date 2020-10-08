/**
 * 大三组
 * Task 1: 基于图片的自瞄流程
 * @author ZhenhuaZhuang
 */
#include <iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
using namespace std;
using namespace cv;

/* 参数定义 */
#define _light_radio 3.2 //灯条长宽比最小值
#define _light_length_min 80.0 //灯条长度最小值
#define _light_length_max 140.0 //灯条长度最大值
#define _light_angle_min 45.0 //灯条角度最小值
#define _light_angle_max 135.0 //灯条角度最大值
#define _armor_angleDiff_ 7 //装甲板左右两侧灯条角差
#define _armor_LenDiff_ratio 0.2 //装甲板左右灯条长度差比
#define _armor_yDiff_ratio_max 2.0 //装甲板左右灯条中心点y的差值比最大值
#define _armor_xDiff_ratio_min 0.5 //装甲板左右灯条中心点x的差值比最小值
#define _armor_radio_min 1.0 //装甲板左右灯条相距距离与灯条长度比值最小值
#define _armor_radio_max 5.0 //装甲板左右灯条相距距离与灯条长度比值最大值

/* 计算两点之间的距离 */
double getDistance(const Point2f& pointO, const Point2f& point1)   
{   
    double distance;  
    distance = powf((pointO.x - point1.x), 2) + powf((pointO.y - point1.y), 2);  
    distance = sqrtf(distance);
    return distance;
}  

/* 绘制旋转矩形 */
void rotatedrectangle(const Mat& src, const RotatedRect& rec, const Scalar& c, int width)
{
   Point2f verticles[4];
   rec.points(verticles);
   for (int i = 0;i < 4;i++)
      line(src, verticles[i], verticles[(i + 1) % 4], c, width);
}

/* 绘制装甲板 */
void paint_armors(const Mat& src, const RotatedRect& leftrec, const RotatedRect& rightrec, const Scalar& c, int width)
{  
   Point2f center = (leftrec.center + rightrec.center) / 2;
   float _length = getDistance(leftrec.center, rightrec.center);
   float _width = _length * 0.8;
   RotatedRect armor = RotatedRect(center, Point2f(_length, _width), leftrec.angle);
   rotatedrectangle(src, armor, c, width);
}

int main()
{
    Mat srcImage, _grayImage1, _grayImage2, binImage1, binImage2, contoursImage, lightsImage, matchImage, armorsImage;
    srcImage = imread("test.jpg");
    if(!srcImage.data) {
       cout << "图像加载失败!" << endl;
       return 0;
    }
    else
       cout << "图像加载成功!" << endl;
    
    contoursImage = srcImage.clone();
    lightsImage = srcImage.clone();
    armorsImage = srcImage.clone();
    matchImage = srcImage.clone();

    /* 进行BGR通道分离 */
    vector<Mat> channels;
    split(srcImage, channels);
    _grayImage1 = channels[2] - channels[0];// Get red - blue image
    _grayImage2 = channels[0];// Get blue image
   
    /* 阈值化 */
    threshold(_grayImage1, binImage1, 218, 255, THRESH_BINARY);
    threshold(_grayImage2, binImage2, 230, 255, THRESH_BINARY);

    namedWindow("二值化", 0);
    imshow("二值化", binImage1);
    imwrite("二值化.jpg", binImage1);

    Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(4, 4));

    /* 膨胀 */
    dilate(binImage1, binImage1, element1);
    dilate(binImage2, binImage2, element2);
    
    vector<RotatedRect> lightInfos1;
    vector<RotatedRect> lightInfos2;

    /* 找红色轮廓 */
    vector<vector<Point>> lightContours1;
    findContours(binImage1.clone(), lightContours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(const auto& contour : lightContours1)
    {
        RotatedRect lightRect = minAreaRect(contour);
        rotatedrectangle(contoursImage, lightRect, Scalar(0, 255, 255), 8);

        Size2f size = lightRect.size;//矩形的边长
        float max = size.width > size.height ? size.width : size.height;//长
        float min = size.width < size.height ? size.width : size.height;//宽
        float angle = (size.width < size.height ? 90 : 0) - lightRect.angle;//角度
        if (max / min <= _light_radio)
            continue;
        if (max <= _light_length_min * srcImage.rows / 3000 || max >= _light_length_max * srcImage.rows / 3000)
            continue;
        if (angle < _light_angle_min || angle > _light_angle_max)
            continue;
        rotatedrectangle(lightsImage, lightRect, Scalar(0, 255, 255), 8);
        lightInfos1.emplace_back(lightRect);
    }

    /* 找蓝色轮廓 */
    vector<vector<Point>> lightContours2;
    findContours(binImage2.clone(), lightContours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(const auto& contour : lightContours2)
    {
        RotatedRect lightRect = minAreaRect(contour);
        rotatedrectangle(contoursImage, lightRect, Scalar(0, 255, 255), 8);

        Size2f size = lightRect.size;     //矩形的边长
        float max = size.width > size.height ? size.width : size.height;//长
        float min = size.width < size.height ? size.width : size.height;//宽
        float angle = (size.width < size.height ? 90 : 0) - lightRect.angle;//角度
        if (max / min <= _light_radio)
            continue;
        if (max <= _light_length_min * srcImage.rows / 3000 || max >= _light_length_max * srcImage.rows / 3000)
            continue;
        if (angle < _light_angle_min || angle > _light_angle_max)
            continue;
        rotatedrectangle(lightsImage, lightRect, Scalar(0, 255, 255), 8);
        lightInfos2.emplace_back(lightRect);
    }
   
    namedWindow("轮廓", 0);
    imshow("轮廓", contoursImage);
    imwrite("轮廓.jpg", contoursImage);

    namedWindow("灯条", 0);
    imshow("灯条", lightsImage);
    imwrite("灯条.jpg", lightsImage);

    /* 按灯条中心x从小到大排序 */
    sort(lightInfos1.begin(), lightInfos1.end(), 
    [](const RotatedRect& ld1, const RotatedRect& ld2) { return ld1.center.x < ld2.center.x; });
    
    vector<RotatedRect> _armors[2];
    _armors[0].clear();
    _armors[1].clear();
    for(size_t i = 0; i < lightInfos1.size(); i++)//遍历所有灯条进行匹配
        for(size_t j = i + 1; j < lightInfos1.size(); j++)
        {
            const RotatedRect& leftLight  = lightInfos1[i];
            const RotatedRect& rightLight = lightInfos1[j];
            //角差
            float angleDiff_ = abs(leftLight.angle - rightLight.angle);
            //长度差比率
            float LenDiff_ratio = abs(leftLight.size.height - rightLight.size.height) / max(leftLight.size.height, rightLight.size.height);
            //筛选
            if (angleDiff_ > _armor_angleDiff_ || LenDiff_ratio > _armor_LenDiff_ratio)
                continue;

            //左右灯条相距距离
            float dis = getDistance(leftLight.center, rightLight.center);
            //左右灯条长度的平均值
            float meanLen = (leftLight.size.height + rightLight.size.height) / 2;
            //左右灯条中心点y的差值
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
            //y差比率
            float yDiff_ratio = yDiff / meanLen;
            //左右灯条中心点x的差值
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
            //x差比率
            float xDiff_ratio = xDiff / meanLen;
            //相距距离与灯条长度比值
            float ratio = dis / meanLen;
            //筛选
            if (yDiff_ratio > _armor_yDiff_ratio_max ||
                xDiff_ratio < _armor_xDiff_ratio_min ||
                ratio > _armor_radio_max ||
                ratio < _armor_radio_min)
                continue;
            rotatedrectangle(matchImage, leftLight, Scalar(0, 255, 255), 8);
            rotatedrectangle(matchImage, rightLight, Scalar(0, 255, 255), 8);
            _armors[0].emplace_back(leftLight);
            _armors[1].emplace_back(rightLight);
        }
    
    for (size_t i = 0; i < _armors[0].size(); i++) 
        paint_armors(armorsImage, _armors[0][i], _armors[1][i], Scalar(0, 0, 255), 8);

    _armors[0].clear();
    _armors[1].clear();
    for(size_t i = 0; i < lightInfos2.size(); i++)//遍历所有灯条进行匹配
        for(size_t j = i + 1; j < lightInfos2.size(); j++)
        {
            const RotatedRect& leftLight  = lightInfos2[i];
            const RotatedRect& rightLight = lightInfos2[j];
            //角差
            float angleDiff_ = abs(leftLight.angle - rightLight.angle);
            //长度差比率
            float LenDiff_ratio = abs(leftLight.size.height - rightLight.size.height) / max(leftLight.size.height, rightLight.size.height);
            //筛选
            if (angleDiff_ > _armor_angleDiff_ || LenDiff_ratio > _armor_LenDiff_ratio)
                continue;

            //左右灯条相距距离
            float dis = getDistance(leftLight.center, rightLight.center);
            //左右灯条长度的平均值
            float meanLen = (leftLight.size.height + rightLight.size.height) / 2;
            //左右灯条中心点y的差值
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
            //y差比率
            float yDiff_ratio = yDiff / meanLen;
            //左右灯条中心点x的差值
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
            //x差比率
            float xDiff_ratio = xDiff / meanLen;
            //相距距离与灯条长度比值
            float ratio = dis / meanLen;
            //筛选
            if (yDiff_ratio > _armor_yDiff_ratio_max ||
                xDiff_ratio < _armor_xDiff_ratio_min ||
                ratio > _armor_radio_max ||
                ratio < _armor_radio_min)
                continue;
            rotatedrectangle(matchImage, leftLight, Scalar(0, 255, 255), 8);
            rotatedrectangle(matchImage, rightLight, Scalar(0, 255, 255), 8);
            _armors[0].emplace_back(leftLight);
            _armors[1].emplace_back(rightLight);
        }
    
    namedWindow("匹配", 0);
    imshow("匹配", matchImage);
    imwrite("匹配.jpg", matchImage);

    for (size_t i = 0; i < _armors[0].size(); i++) 
        paint_armors(armorsImage, _armors[0][i], _armors[1][i], Scalar(255, 0, 0), 8);

    /* 输出并保存结果 */
    namedWindow("装甲板", 0);
    imshow("装甲板", armorsImage);
    imwrite("armor.jpg", armorsImage);
    waitKey(0);
    return 0;
}