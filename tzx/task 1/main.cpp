/**
 * 大三组
 * Task 1: 基于图片的自瞄流程
 * @author frezcirno
 */

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;

class MyRotatedRect : public RotatedRect
{
public:
    using RotatedRect::RotatedRect;
    MyRotatedRect() = default;
    MyRotatedRect(const MyRotatedRect &x) = default;
    MyRotatedRect(const RotatedRect &base) : RotatedRect(base) {}

    float height() const { return _height ? _height : _height = std::max(this->size.height, this->size.width); }
    float width() const { return _width ? _width : _width = std::min(this->size.height, this->size.width); }
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
        // putText(img, std::to_string(this->size.area()), vertices[0], FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(66, 0xcc, 0xff)); // for debug
    }

private:
    mutable float _height = 0, _width = 0, _angleT = 0, _aspectRatio = 0, _area = 0;
};

struct Armor : MyRotatedRect
{
    using MyRotatedRect::MyRotatedRect;
    std::vector<MyRotatedRect> lights;
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

std::vector<MyRotatedRect> findContourRect(Mat hsvImg, InputArray lowerb, InputArray upperb)
{
    Mat mask;
    inRange(hsvImg, lowerb, upperb, mask);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    std::vector<MyRotatedRect> contourRects;
    for (auto &&contour : contours)
    {
        auto rect = minAreaRect(contour);
        if (rect.size.area() < hsvImg.size[0] * hsvImg.size[1] / 1000000.0)
        {
            continue;
        }
        contourRects.emplace_back(rect);
    }

    return contourRects;
}

Scalar hsvToOpencvHsv(double h, double s, double v)
{
    return {h / 2, s * 255, v * 255};
}

float euclideanDistSquare(const Point &p, const Point &q)
{
    Point diff = p - q;
    return sqrt(diff.x * diff.x + diff.y * diff.y);
}

std::vector<Armor> findArmor(const std::vector<MyRotatedRect> &lights)
{
    std::vector<Armor> armors;
    for (int i = 0; i < lights.size(); i++)
    {
        for (int j = i + 1; j < lights.size(); j++)
        {
            auto light0 = lights[i], light1 = lights[j];

            float angleDiff = std::abs(light0.angleT() - light1.angleT());
            if (angleDiff > 5)
            {
                continue;
            }

            float aspectRatioDiff = std::abs(light0.aspectRatio() - light1.aspectRatio());
            if (aspectRatioDiff > 5)
            {
                continue;
            }

            float centerDist = euclideanDistSquare(light0.center, light1.center);
            float avgheight = (light0.height() + light1.height()) / 2;
            float aspectRatio = centerDist / avgheight;
            if (aspectRatio > 4)
            {
                continue;
            }
            armors.emplace_back((light0.center + light1.center) / 2,
                                Size(centerDist, avgheight * 2),
                                light0.angle);
            armors.back().lights.push_back(light0);
            armors.back().lights.push_back(light1);
        }
    }
    return armors;
}

int main()
{
    Mat img = imread("test.png");
    resize(img, img, Size(1600, 1200));

    Mat tmpImg;
    cvtColor(img, tmpImg, COLOR_BGR2GRAY);
    threshold(tmpImg, tmpImg, 200, 255, THRESH_BINARY);
    imshow("1-二值化", tmpImg);
    imwrite("1-二值化.jpg", tmpImg);
    waitKey();

    Mat hsvImg;
    cvtColor(img, hsvImg, COLOR_BGR2HSV);

    std::vector<MyRotatedRect> contours[2] = {
        findContourRect(hsvImg, hsvToOpencvHsv(25, 0.1, 0.9), hsvToOpencvHsv(50, 1, 1)),
        findContourRect(hsvImg, hsvToOpencvHsv(170, 0, 0.7), hsvToOpencvHsv(220, 0.6, 1))};

    std::vector<MyRotatedRect> lights[2];

    tmpImg = img.clone();
    for (int i = 0; i < sizeof(contours) / sizeof(contours[0]); i++)
    {
        for (auto &&contour : contours[i])
        {
            contour.draw(tmpImg, Scalar(0, 255, 255), 3);
            if (isLedLight(contour, img))
            {
                lights[i].push_back(contour);
            }
        }
    }
    imshow("2-找到轮廓", tmpImg);
    imwrite("2-找到轮廓.jpg", tmpImg);
    waitKey();

    tmpImg = img.clone();
    for (int i = 0; i < sizeof(lights) / sizeof(lights[0]); i++)
    {
        for (auto &&light : lights[i])
        {
            light.draw(tmpImg, Scalar(0, 255, 255), 3);
        }
    }
    imshow("3-筛选轮廓", tmpImg);
    imwrite("3-筛选轮廓.jpg", tmpImg);
    waitKey();

    for (auto &&light : lights[0])
    {
        light.draw(img, Scalar(0, 255, 0), 3);
    }
    for (auto &&light : lights[1])
    {
        light.draw(img, Scalar(0, 0, 255), 3);
    }
    imshow("4-找到灯条", img);
    imwrite("4-找到灯条.jpg", img);
    waitKey();

    std::vector<Armor> armors[] = {findArmor(lights[0]),
                                   findArmor(lights[1])};

    tmpImg = img.clone();
    for (int i = 0; i < sizeof(armors) / sizeof(armors[0]); i++)
    {
        for (auto &&armor : armors[i])
        {
            auto randColor = []() { return rand() % 255; };
            auto color = Scalar(randColor(), randColor(), randColor());
            armor.lights[0].draw(tmpImg, color, 3);
            armor.lights[1].draw(tmpImg, color, 3);
        }
    }
    imshow("5-配对灯条", tmpImg);
    imwrite("5-配对灯条.jpg", tmpImg);
    waitKey();

    for (auto &&armor : armors[0])
    {
        armor.draw(img, Scalar(255, 255, 0), 3);
    }
    for (auto &&armor : armors[1])
    {
        armor.draw(img, Scalar(255, 0, 255), 3);
    }
    imshow("6-装甲板位置", img);
    imwrite("armor.jpg", img);
    waitKey();

    return 0;
}
