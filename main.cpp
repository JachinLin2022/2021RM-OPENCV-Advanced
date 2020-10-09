#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;

void CallBack_Demo(int, void*)
{
    return;
}


void get_lights_mask(cv::InputArray img, cv::OutputArray mask)
{
    cv::Mat imgHSV, imgBlur, maskBlur;;
    int h_min = 0, h_max = 255, s_min = 0, s_max = 159, v_min = 231, v_max = 255;

    // 转HSV格式
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);
    // 高斯模糊
    cv::GaussianBlur(imgHSV, imgBlur, cv::Size(9, 9), 0);
    // 以指定范围的阈值进行二值化
    cv::inRange(imgBlur, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), maskBlur);
    // 优化处理
    // 模糊
    cv::GaussianBlur(maskBlur, maskBlur, cv::Size(7, 7), 0);
    // 闭操作填充黑点
    cv::morphologyEx(maskBlur, mask, cv::MORPH_CLOSE, cv::Mat(9, 9, CV_8U));


}

void get_lights_contours(cv::InputOutputArray mask, vector<vector<cv::Point>> &contours)
{
    // 提取轮廓
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
}

void get_lights_rects(vector<vector<cv::Point>> contours, vector<cv::RotatedRect> &rects)
{
    for (int i = 0; i < contours.size(); i++) {

        double area = contourArea(contours[i], true);

        //cout << area << endl;
        if (abs(area) > 10.0) {

            //cv::drawContours(imgContours, contours, i, cv::Scalar(255, 0, 0), 3, 8);
            cv::RotatedRect r = cv::minAreaRect(contours[i]);

            cv::Point2f vertices[4];
            r.points(vertices);
            float w, h;
            w = sqrt(powf((vertices[0].x - vertices[1].x), 2) + powf((vertices[0].y - vertices[1].y), 2));
            h = sqrt(powf((vertices[1].x - vertices[2].x), 2) + powf((vertices[1].y - vertices[2].y), 2));

            if ((w / h >= 2.0) && (w * h > 500.0)) {
                rects.push_back(r);
                //cv::fillPoly(img, contours, cv::Scalar(255, 0, 255));

                cout << w << " " << h << endl;
                cout << w * h << " " << r.angle << endl;
            }

        }

    }
}

void show_lights_rects(cv::InputOutputArray &img, vector<cv::RotatedRect> rects)
{
    for (int i = 0; i < rects.size(); i++) {
        cv::Point2f vertices[4];
        // 取点
        rects[i].points(vertices);

        for (int l = 0; l < 4; l++)
            cv::line(img, vertices[l], vertices[(l + 1) % 4], cv::Scalar(255, 0, 255), 5);
    }
}

void find_short_side(cv::RotatedRect rect, cv::Point &p1, cv::Point &p2)
{
    int pos;
    double min = 100000000.0;
    cv::Point2f points[4];
    rect.points(points);
    for (int i = 0; i < 4; i++) {
        double tmp = (points[i].x - points[(i + 1) % 4].x)* (points[i].x - points[(i + 1) % 4].x) + (points[i].y - points[(i + 1) % 4].y) * (points[i].y - points[(i + 1) % 4].y);
        if (tmp < min) {
            min = tmp;
            pos = i;
        }
    }
    int x1, x2, y1, y2;
    x1 = (points[pos].x + points[(pos + 1) % 4].x) / 2;
    y1 = (points[pos].y + points[(pos + 1) % 4].y) / 2;
    x2 = (points[(pos + 2) % 4].x + points[(pos + 3) % 4].x) / 2;
    y2 = (points[(pos + 2) % 4].y + points[(pos + 3) % 4].y) / 2;
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

bool judging_rect(cv::RotatedRect rect1, cv::RotatedRect rect2)
{
    cv::Point points[2][2];
    find_short_side(rect1, points[0][0], points[0][1]);
    find_short_side(rect2, points[1][0], points[1][1]);
    double threshold = 1;
    double angle, angle1, angle2, dis_width, dis_length;
    if (fabs(points[0][0].x - points[1][0].x) < 1e-6) {
        angle1 = 90;
    }
    else {
        angle1 = atan((points[0][0].y - points[1][0].y) / points[0][0].x - points[1][0].x);
    }
    if (fabs(points[0][0].x - points[0][1].x) < 1e-6) {
        angle2 = 90;
    }
    else {
        angle2 = atan((points[0][1].y - points[1][1].y) / points[0][1].x - points[1][1].x);
    }
    dis_width = sqrt((points[0][0].x - points[0][1].x) * (points[0][0].x - points[0][1].x) + (points[0][0].y - points[0][1].y) * (points[0][0].y - points[0][1].y));
    dis_length = sqrt((points[0][0].x - points[1][0].x) * (points[0][0].x - points[1][0].x) + (points[0][0].y - points[1][0].y) * (points[0][0].y - points[1][0].y));
    int x1 = points[0][1].x - points[0][0].x;
    int y1 = points[0][1].y - points[0][0].y;
    int x2 = points[1][0].x - points[0][0].x;
    int y2 = points[1][0].y - points[0][0].y;
    angle = cos((x1 * x2 + y1 * y2) / (dis_width * dis_length));
    if (angle < threshold && dis_length / dis_width > 1 && dis_length / dis_width < 3.5)
        return true;
    else
        return false;
}

void matching_lights(const vector<cv::RotatedRect>& r_rects, vector<vector<int>>& matched_pairs)
{
    double angle_threshold = 5.0;
    for (int i = 0; i<int(r_rects.size()); i++) {
        vector<int> tmp_record;
        for (int j = 0; j<int(r_rects.size()); j++) {
            if (j == i)
                continue;
            if (fabs(r_rects[i].angle - r_rects[j].angle) < angle_threshold) {
                if (judging_rect(r_rects[i], r_rects[j]))
                    tmp_record.push_back(j);
            }
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
            vector<int> tmp;
            tmp.push_back(i);
            tmp.push_back(pos);
            matched_pairs.push_back(tmp);
        }
    }
}


int main(int argc, char** argv)
{
    cv::Mat img, mask;
    vector<vector<cv::Point>> contours;
    vector<cv::RotatedRect> rects;

    img = cv::imread("../test.jpg");
    if (img.empty()){
        cout << "test.jpg can't open!" << endl;
        return -1;
    }

    // 获取灯条二值化掩膜
    get_lights_mask(img, mask);
    // 获取掩膜轮廓
    get_lights_contours(mask, contours);
    // 获取灯条矩形
    get_lights_rects(contours, rects);
    // 打印灯条矩形
    //show_lights_rects(img, rects);

    vector<vector<int>> matched_pairs;
    vector<cv::Point> centers;
    matching_lights(rects, matched_pairs);

    for (int i = 0; i<int(matched_pairs.size()); i++) {
        cv::Point points[2][2];
        find_short_side(rects[matched_pairs[i][0]], points[0][0], points[0][1]);
        find_short_side(rects[matched_pairs[i][1]], points[1][0], points[1][1]);
        cv::Point center;
        center.x = (points[0][0].x + points[1][1].x) / 2;
        center.y = (points[0][0].y + points[1][1].y) / 2;
        centers.push_back(center);
        cv::circle(img, center, 20, cv::Scalar(0, 255, 255), 10);
        cv::Point2f vertices[4];
        rects[matched_pairs[i][0]].points(vertices);
        for (int l = 0; l < 4; l++)
            line(img, vertices[l], vertices[(l + 1) % 4], cv::Scalar(255, 0, 255), 6);
        rects[matched_pairs[i][1]].points(vertices);
        for (int l = 0; l < 4; l++)
            line(img, vertices[l], vertices[(l + 1) % 4], cv::Scalar(255, 0, 255), 6);
    }


    cout << rects.size() << endl;

//    cv::namedWindow("mask", 0);
//    cv::resizeWindow("mask", cv::Size(800, 600));
//    imshow("mask", mask);

    cv::namedWindow("Original", 0);
    cv::resizeWindow("Original", cv::Size(800, 600));
    imshow("Original", img);

    cv::imwrite("armor.jpg", img);
    cv::waitKey(0);

    return 0;
}