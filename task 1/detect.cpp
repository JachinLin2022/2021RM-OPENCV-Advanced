#include <vector>
#include <iostream>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Parameters
const double MIN_RECT_AREA = 50;
const double MIN_WH_RATIO = 2;    // Min Ratio of width & height
const double MAX_WH_RATIO = 10;   // Max Ratio of width & height
const double MIN_AREA_RATIO = 0.1;// Min Ratio of area & rect
const double Norm2 = 9;           // The Second Norm (isSameBar)
const int light_threshold = 229;  // Light threshold (Red 233 Blue 222)
const int dim_threshold = 255;    // Dim threshold
const int ENEMY_COLOR = 2;        // For Blue is 0, Red is 2
const double ANGEL_RANGE = 5;     // Angel threshold of two light bars (Red 5 Blue 2.5)
const double HEIGHT_RANGE = 25;   // Height threshold of two light bars (Red 30 Blue)

class LightBar
{
public:
    cv::RotatedRect rect; // bar pos
    double area_ratio;
    double length; // bar length

    LightBar(cv::RotatedRect &r, double ratio) : rect(r), area_ratio(ratio)
    {
        length = std::max(rect.size.height, rect.size.width);
    }
    LightBar() = default;
};
typedef std::vector<LightBar> LightBars;

class ArmorBox
{
public:
    cv::Rect rect;
    LightBars light_bars;

    ArmorBox(const cv::Rect &pos, const LightBars &bars) : rect(pos), light_bars(bars) {}
};
typedef std::vector<ArmorBox> ArmorBoxes;

// Ratio of Widht & Height
static double WH_Ratio(const cv::RotatedRect &rect)
{
    return rect.size.height > rect.size.width ? rect.size.height / rect.size.width : rect.size.width / rect.size.height;
}

// Ratio of area of contour & area of rectangle
static double Area_Ratio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect)
{
    return cv::contourArea(contour) / rect.size.area();
}

// Ratio of Width,Height inrange && Area of rect, Area of ratio inRange
static bool isValidLightBar(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect)
{
    return (MIN_WH_RATIO < WH_Ratio(rect) && WH_Ratio(rect) < MAX_WH_RATIO) &&
           ((rect.size.area() < MIN_RECT_AREA && Area_Ratio(contour, rect) > MIN_AREA_RATIO) ||
            (rect.size.area() >= MIN_RECT_AREA && Area_Ratio(contour, rect) > MIN_AREA_RATIO));
}

static bool isSameBar(LightBar bar1, LightBar bar2)
{
    auto dist = bar1.rect.center - bar2.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < Norm2;
}

// Judging Angle of two bars
static bool angelJudge(const LightBar &light_bar_i, const LightBar &light_bar_j)
{
    double angle_i = light_bar_i.rect.size.width > light_bar_i.rect.size.height ? light_bar_i.rect.angle : light_bar_i.rect.angle - 90;
    double angle_j = light_bar_j.rect.size.width > light_bar_j.rect.size.height ? light_bar_j.rect.angle : light_bar_j.rect.angle - 90;
    return std::abs(angle_i - angle_j) < ANGEL_RANGE;
}

// Judging Height of two bars
static bool heightJudge(const LightBar &light_bar_i, const LightBar &light_bar_j)
{
    cv::Point2f centers = light_bar_i.rect.center - light_bar_j.rect.center;
    return abs(centers.y) < HEIGHT_RANGE;
}

// Judging pair of two bars
static bool isPairLights(const LightBar &light_blob_i, const LightBar &light_blob_j)
{
    return heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j);
}

// opening & closing (morphology)
static void imagePreProcess(cv::Mat &srcImage)
{
    static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(srcImage, srcImage, kernel_erode);

    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(srcImage, srcImage, kernel_dilate);

    static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(srcImage, srcImage, kernel_dilate2);

    static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(srcImage, srcImage, kernel_erode2);
}

bool find_light_bar(const cv::Mat &srcImage, LightBars &light_bars)
{
    cv::Mat color_channel, src_light, src_dim;
    std::vector<cv::Mat> channels;

    cv::split(srcImage, channels);
    color_channel = channels[ENEMY_COLOR]; // Enemy channel

    cv::threshold(color_channel, src_light, light_threshold, 255, CV_THRESH_BINARY);
    if (src_light.empty())
        return false;
    imagePreProcess(src_light);
    // cv::imshow("Light", src_light);

    cv::threshold(color_channel, src_dim, dim_threshold, 255, CV_THRESH_BINARY);
    if (src_dim.empty())
        return false;
    imagePreProcess(src_dim);
    // cv::imshow("Dim", src_dim);

    std::vector<std::vector<cv::Point>> light_contours_light, light_contour_dim;
    LightBars light_bars_light, light_bars_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
    cv::findContours(src_light, light_contours_light, hierarchy_light, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    cv::findContours(src_dim, light_contour_dim, hierarchy_dim, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

    for (int i = 0; i < light_contours_light.size(); i++)
    {
        if (hierarchy_light[i][2] == -1) // no parent element
        {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            if (isValidLightBar(light_contours_light[i], rect))
            {
                light_bars_light.emplace_back(rect, Area_Ratio(light_contours_light[i], rect));
            }
        }
    }

    for (int i = 0; i < light_contour_dim.size(); i++)
    {
        if (hierarchy_dim[i][2] == -1) // no parent element
        {
            cv::RotatedRect rect = cv::minAreaRect(light_contour_dim[i]);
            if (isValidLightBar(light_contour_dim[i], rect))
            {
                light_bars_dim.emplace_back(rect, Area_Ratio(light_contour_dim[i], rect));
            }
        }
    }

    std::vector<int> light_to_rm, dim_to_rm;
    for (int l = 0; l < light_bars_light.size(); l++)
    {
        for (int d = 0; d < light_bars_dim.size(); d++)
        {
            if (isSameBar(light_bars_light[l], light_bars_dim[d]))
            {
                if (light_bars_light[l].area_ratio > light_bars_dim[d].area_ratio)
                {
                    dim_to_rm.emplace_back(d);
                }
                else
                    light_to_rm.emplace_back(l);
            }
        }
    }

    sort(light_to_rm.begin(), light_to_rm.end(), [](int a, int b) { return a > b; });
    sort(dim_to_rm.begin(), dim_to_rm.end(), [](int a, int b) { return a > b; });

    for (auto i : light_to_rm)
    {
        light_bars_light.erase(light_bars_light.begin() + i);
    }
    for (auto i : dim_to_rm)
    {
        light_bars_dim.erase(light_bars_dim.begin() + i);
    }
    for (const auto &light : light_bars_light)
    {
        light_bars.emplace_back(light);
    }
    for (const auto &dim : light_bars_dim)
    {
        light_bars.emplace_back(dim);
    }

    return light_bars.size() >= 2;
}

bool find_armor_boxes(const cv::Mat &srcImage, const LightBars &light_bars, ArmorBoxes &armorboxes)
{
    for (int i = 0; i < light_bars.size(); i++)
    {
        for (int j = i + 1; j < light_bars.size(); j++)
        {
            if (!isPairLights(light_bars.at(i), light_bars.at(j)))
                continue;
            cv::Rect2d rect_left = light_bars.at(i).rect.boundingRect();
            cv::Rect2d rect_right = light_bars.at(j).rect.boundingRect();
            double min_x, max_x, min_y, max_y;
            min_x = std::fmin(rect_left.x, rect_right.x) - 4;
            max_x = std::fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
            min_y = std::fmin(rect_left.y, rect_right.y) - (rect_left.height + rect_right.height) / 4;
            max_y = std::fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + (rect_left.height + rect_right.height) / 4;
            if (min_x < 0 || max_x > srcImage.cols || min_y < 0 || min_y > srcImage.rows)
                continue;
            LightBars pair_bars = {light_bars.at(i), light_bars.at(j)};
            armorboxes.emplace_back(cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y), pair_bars);
        }
    }
    return !armorboxes.empty();
}

int main()
{
    cv::Mat srcImage = cv::imread("test.jpg");
    if (!srcImage.empty())
    {
        cv::Mat resizeImage;
        LightBars lightbars;
        ArmorBoxes armorboxes;
        cv::resize(srcImage, resizeImage, cv::Size(srcImage.cols / 4, srcImage.rows / 4), 0, 0, CV_INTER_LINEAR);

        find_light_bar(resizeImage, lightbars);
/*         
        for (const auto lightbar : lightbars)
        {
            cv::Point2f points[4];
            lightbar.rect.points(points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(resizeImage, points[i], points[(i + 1) % 4], ENEMY_COLOR == 0 ? cv::Scalar(200, 0, 0) : cv::Scalar(0, 0, 200), 2);
            }
        }
 */
        find_armor_boxes(resizeImage, lightbars, armorboxes);
        for (const auto armorbox : armorboxes)
        {
            cv::rectangle(resizeImage, armorbox.rect, ENEMY_COLOR == 0 ? cv::Scalar(200, 0, 0) : cv::Scalar(0, 0, 200), 2);
        }

        cv::imshow("Result", resizeImage);
        // cv::imwrite("Result_Red.jpg", resizeImage);
        cv::waitKey(0);
    }
    return 0;
}