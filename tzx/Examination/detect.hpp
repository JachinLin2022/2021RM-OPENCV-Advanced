/**
 * 大三组
 * Examination: Detection Module
 * @author frezcirno
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MyRotatedRect : public RotatedRect
{
public:
    using RotatedRect::RotatedRect;
    MyRotatedRect() = default;
    MyRotatedRect(const MyRotatedRect &x) = default;
    MyRotatedRect(const RotatedRect &base) : RotatedRect(base) {}

    float height() const { return _height ? _height : _height = max(this->size.height, this->size.width); }
    float width() const { return _width ? _width : _width = min(this->size.height, this->size.width); }
    float angleT() const { return _angleT ? _angleT : _angleT = (this->size.height > this->size.width ? 90 : 0) - RotatedRect::angle; }
    float aspectRatio() const { return _aspectRatio ? _aspectRatio : _aspectRatio = height() / width(); }
    float area() const { return _area ? _area : _area = RotatedRect::size.area(); }

    void draw(Mat img, const Scalar &color, int thickness = 1)
    {
        Point2f vertices[4];
        this->points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
        }

        // putText(img, to_string(debugInfo), vertices[0], FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(66, 0xcc, 0xff)); // for debug
    }

private:
    mutable float _height = 0, _width = 0, _angleT = 0, _aspectRatio = 0, _area = 0;
};

struct Light : MyRotatedRect
{
    using MyRotatedRect::MyRotatedRect;
    struct Armor* belongTo = NULL;
};

struct Armor : MyRotatedRect
{
    using MyRotatedRect::MyRotatedRect;
    vector<Light> lights;
    void addLight(Light& light){
        light.belongTo = this;
        lights.push_back(light);
    }
};

inline int euclideanDistSquare(const Point &p, const Point &q)
{
    Point diff = p - q;
    return diff.x * diff.x + diff.y * diff.y;
}

bool isLedLight(const vector<Point> &contour, const Mat &img, MyRotatedRect &resultRect)
{
    auto rect = (MyRotatedRect)minAreaRect(contour);

#if 1
    // 长宽比过滤
    if (rect.aspectRatio() < 3)
    {
        return false;
    }
#endif

#if 1
    // 倾斜角度过滤
    if (rect.angleT() < 45 || rect.angleT() > 135)
    {
        return false;
    }
#endif

#if 1
    // 面积过滤
    double areaRatio = rect.area() / img.size().area();
    if (areaRatio < 0.00005)
    {
        return false;
    }
#endif

#if 1
    // 占比过滤
    double cArea = contourArea(contour, false);
    double cAreaRatio = cArea / rect.area();
    if (isnan(cAreaRatio) || cAreaRatio < 0.7) { 
        return false; 
    }
#endif

    resultRect = rect;
    return true;
}

vector<Light> findLights(Mat hsvImg, InputArray lowerb, InputArray upperb)
{
    Mat mask;
    inRange(hsvImg, lowerb, upperb, mask);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Light> lights;
    for (auto &&contour : contours)
    {
        MyRotatedRect resultRect;
        if (isLedLight(contour, hsvImg, resultRect))
        {
            lights.push_back(resultRect);
        }
    }

    return lights;
}

bool isArmor(const Light &light0, const Light &light1)
{
#if 1
    // 角度差值
    float angleDiff = abs(light0.angleT() - light1.angleT());
    if (angleDiff > 10) { 
        return false; 
    }
#endif

#if 1
    // 长宽比差值
    float aspectRatioDiff = abs(light0.aspectRatio() - light1.aspectRatio());
    if (aspectRatioDiff > 5) { 
        return false; 
    }
#endif

#if 1
    // 距离
    int centerDistSquare = euclideanDistSquare(light0.center, light1.center);
    float avgheight = (light0.height() + light1.height()) / 2;
    double distRatioSquare = centerDistSquare / avgheight / avgheight;
    if (distRatioSquare < 3 || distRatioSquare > 12) { 
        return false; 
    }
#endif

#if 1
    // x轴, y轴距离
    int xDist = abs(light0.center.x - light1.center.x);
    int yDist = abs(light0.center.y - light1.center.y);
    if (xDist < 2 * avgheight || yDist > 2 * avgheight) { 
        return false; 
    }
#endif

    return true;
}

/**
 * @brief 从一堆灯条里面找出装甲板
 * 
 * @param lights
 * @return vector<Armor> 
 */
vector<Armor> findArmor(vector<Light> &lights)
{
    vector<Armor> armors;
    for (int i = 0; i < lights.size(); i++)
    {
        auto& light0 = lights[i];
        if (light0.belongTo) { 
            continue; 
        }

        for (int j = i + 1; j < lights.size(); j++) 
        {
            auto& light1 = lights[j];
            if (light1.belongTo) {
                continue; 
            }

            if (isArmor(light0, light1)) {
                float avgheight = (light0.height() + light1.height()) / 2;
                armors.emplace_back((light0.center + light1.center) / 2,
                    Size(avgheight * 2, avgheight * 2),
                    -(light0.angleT() + light1.angleT()) / 2);
                auto back = armors.back();
                back.addLight(light0);
                back.addLight(light1);
                break;
            }
        }
    }
    return armors;
}

/**
 * @brief 识别总函数
 * 
 * @param img 原图
 * @param lights 返回值
 * @param armors 返回值
 */
void detect(Mat &img, vector<vector<Light>>& lights, vector<vector<Armor>>& armors)
{
    Mat hsvImg;
    cvtColor(img, hsvImg, COLOR_BGR2HSV);

    lights = {findLights(hsvImg, Scalar{12, 25, 229}, Scalar{25, 255, 255}),
              findLights(hsvImg, Scalar{85, 0, 178}, Scalar{110, 153, 255})};

    armors = {findArmor(lights[0]), findArmor(lights[1])};
}
