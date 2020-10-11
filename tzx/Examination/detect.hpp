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
        // putText(img, to_string(this->size.area()), vertices[0], FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(66, 0xcc, 0xff)); // for debug
    }

private:
    mutable float _height = 0, _width = 0, _angleT = 0, _aspectRatio = 0, _area = 0;
};

struct Armor : MyRotatedRect
{
    using MyRotatedRect::MyRotatedRect;
    vector<MyRotatedRect> lights;
};

bool isLedLight(const MyRotatedRect &rect, const Mat &img)
{
    if (rect.angleT() < 45 || rect.angleT() > 135)
    {
        return false;
    }

    if (rect.area() < 800.0 * img.size[0] * img.size[1] / 4497 / 3000)
    {
        return false;
    }

    if (rect.aspectRatio() < 3)
    {
        return false;
    }
    return true;
}

vector<MyRotatedRect> findLights(Mat hsvImg, InputArray lowerb, InputArray upperb)
{
    Mat mask;
    inRange(hsvImg, lowerb, upperb, mask);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    vector<MyRotatedRect> lights;
    for (auto &&contour : contours)
    {
        auto rect = minAreaRect(contour);
        if (rect.size.area() < hsvImg.size[0] * hsvImg.size[1] / 1000000.0)
        {
            continue;
        }
        if (isLedLight(rect, hsvImg))
        {
            lights.push_back(rect);
        }
    }

    return lights;
}

inline float euclideanDistSquare(const Point &p, const Point &q)
{
    Point diff = p - q;
    return diff.x * diff.x + diff.y * diff.y;
}

vector<Armor> findArmor(const vector<MyRotatedRect> &lights)
{
    vector<Armor> armors;
    for (int i = 0; i < lights.size(); i++)
    {
        for (int j = i + 1; j < lights.size(); j++)
        {
            auto light0 = lights[i], light1 = lights[j];

            float angleDiff = abs(light0.angleT() - light1.angleT());
            if (angleDiff > 5)
            {
                continue;
            }

            float aspectRatioDiff = abs(light0.aspectRatio() - light1.aspectRatio());
            if (aspectRatioDiff > 5)
            {
                continue;
            }

            float centerDistSquare = euclideanDistSquare(light0.center, light1.center);
            float avgheight = (light0.height() + light1.height()) / 2;
            float avgheightSquare = avgheight * avgheight;
            if (centerDistSquare / avgheightSquare > 16)
            {
                continue;
            }
            armors.emplace_back((light0.center + light1.center) / 2,
                                Size(avgheight * 3, avgheight * 2),
                                light0.angle);
            armors.back().lights.push_back(light0);
            armors.back().lights.push_back(light1);
        }
    }
    return armors;
}

vector<vector<Armor>> detect(Mat &img)
{
    Mat hsvImg;
    cvtColor(img, hsvImg, COLOR_BGR2HSV);

    vector<MyRotatedRect> lights[] = {findLights(hsvImg, Scalar{12, 25, 229}, Scalar{25, 255, 255}),
                                      findLights(hsvImg, Scalar{85, 0, 178}, Scalar{110, 153, 255})};

    for (auto &&light : lights[0])
    {
        light.draw(img, Scalar(255, 255, 0), 3);
    }
    for (auto &&light : lights[1])
    {
        light.draw(img, Scalar(255, 0, 255), 3);
    }

    vector<vector<Armor>> armors = {findArmor(lights[0]),
                                    findArmor(lights[1])};

    for (auto &&armor : armors[0])
    {
        armor.draw(img, Scalar(255, 255, 0), 3);
    }
    for (auto &&armor : armors[1])
    {
        armor.draw(img, Scalar(255, 0, 255), 3);
    }

    return armors;
}
