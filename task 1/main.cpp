#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"  
#include <iostream>
#include<vector>
#include<cstring>
#include<cstdio>
#include<fstream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
//一个随机颜色的序列，用来在标注轮廓的时候上色
Scalar clrs[10]={Scalar(255,0,255),Scalar(205,41,144),Scalar(132,112,255),Scalar(0,255,255),Scalar(255,255,0),Scalar(244,164,95)};
int minh,maxh,mins,maxs,minv,maxv;
//当使用下列两个函数的时候，分别调整阈值到红灯和蓝灯
void adjust_to_filter_red()
{
    minh = 8;
    maxh = 23;
    mins = 0;
    maxs = 255;
    minv = 200;
    maxv = 255;
}
void adjust_to_filter_blue()
{
    minh = 85;
    maxh = 105;
    mins = 0;
    maxs = 150;
    minv = 180;
    maxv = 255;
}
//用来判断是否轮廓是灯条
bool ifDengtiao(RotatedRect& target)
{
    //判断的方式是长宽比
    double mianji=target.size.width*target.size.height;
    if(mianji>800.0&&mianji<=4000.0)
    {
        double f1=target.size.width;
        double f2=target.size.height;
        if(f1<f2)
        std::swap(f1,f2);
        double rate=f1/f2;
        if(rate>=(3.2))
        return true;
    }
    return false;
    
}
//引入一个简单的计算几何模板，主要用来判断直线夹角
#define Vector MyPoint  
const double eps = 1e-6;  
int dcmp(double x) { return fabs(x) < eps ? 0 : (x < 0 ? -1 : 1); }  
  
struct MyPoint {  
    double x, y;  
  
    MyPoint(const MyPoint& rhs): x(rhs.x), y(rhs.y) { } //拷贝构造函数  
    MyPoint(double x = 0.0, double y = 0.0): x(x), y(y) { }   //构造函数  
  
    friend istream& operator >> (istream& in, MyPoint& P) { return in >> P.x >> P.y; }  
    friend ostream& operator << (ostream& out, const MyPoint& P) { return out << P.x << ' ' << P.y; }  
  
    friend Vector operator + (const Vector& A, const Vector& B) { return Vector(A.x+B.x, A.y+B.y); }  
    friend Vector operator - (const MyPoint& A, const MyPoint& B) { return Vector(A.x-B.x, A.y-B.y); }  
    friend Vector operator * (const Vector& A, const double& p) { return Vector(A.x*p, A.y*p); }  
    friend Vector operator / (const Vector& A, const double& p) { return Vector(A.x/p, A.y/p); }  
    friend bool operator == (const MyPoint& A, const MyPoint& B) { return dcmp(A.x-B.x) == 0 && dcmp(A.y-B.y) == 0; }  
    friend bool operator < (const MyPoint& A, const MyPoint& B) { return A.x < B.x || (A.x == B.x && A.y < B.y); }  
  
    void in(void) { scanf("%lf%lf", &x, &y); }  
    void out(void) { printf("%lf %lf", x, y); }  
};  
  
#define PI 3.14159265
const double ConvertToJiaodu=180.0 / PI;
double MyDot(const Vector& A, const Vector& B) { return A.x*B.x + A.y*B.y; }  //点积  
double MyLength(const Vector& A){ return sqrt(MyDot(A, A)); }  
double MyAngle(const Vector& A, const Vector& B) { return acos(MyDot(A, B)/MyLength(A)/MyLength(B))*ConvertToJiaodu; }  //向量夹角  
/*
输入为一个旋转矩形
输出为一个向量（计算几何模板中定义），代表该旋转矩形长轴的方向，仅代表直线方向，向量正负没有实际意义

*/
Vector mainVecOfRect(RotatedRect& r1)
{
    Point2f vers[4];//四个顶点
    r1.points(vers);
    Vector xs[2]={{0,0},{0,0}};//存储两个向量，比较长度
    for(int i=0;i<=1;i++)
    {
        MyPoint ds(vers[i].x,vers[i].y);
        MyPoint ed(vers[i+1].x,vers[i+1].y);
        Vector biu(ds.x-ed.x,ds.y-ed.y);
        xs[i]=biu;
    }
    if(MyLength(xs[0])>MyLength(xs[1]))
    return xs[0];
    else
    return xs[1];
}
/*
输入：已经判定为灯条的两个旋转矩形轮廓
输出：是否是配对的灯条
*/
bool pairing(RotatedRect& x1,RotatedRect& x2)
{
    double deltaangle=abs(x1.angle-x2.angle);//旋转角度差，如果相近可能是方向垂直或者平行
    
   if(deltaangle<1.8)
   {
       Vector s1=mainVecOfRect(x1),s2=mainVecOfRect(x2);
       //如果长轴互相垂直的话，不配对
       if(fabs((MyAngle(s1,s2)-90))<5.0)
       return false;
        Vector zhou(x1.center.x-x2.center.x,x1.center.y-x2.center.y);
        //如果两个矩形中心连线并不垂直于他们的长轴的话，那么不是矩形的轮廓，也不配对
        if(fabs(MyAngle(zhou,s1)-90)>10)
        return false;
       return true;
   }
   else
   {
       return false;
   }
   
}
/*
输入：一个RGB通道彩色图片，绘画颜色，需要绘画的旋转矩形，绘画的线条长度
在目标图片上绘画目标颜色的矩形
*/
void StrokeRectOnMat(Mat& des,Scalar Color,RotatedRect tar,int thick)
{
     Point2f vers[4];
      tar.points(vers);
      for(int j=0;j<4;j++)
             {
            line(des,vers[j],vers[(j+1)%4],Color,thick);       
             }
        
}
/*
输入：两个坐标点
输出：两点连线的中点的坐标
*/
MyPoint middlePoint(MyPoint xxx,MyPoint yyy)
{
    Vector biu(xxx.x-yyy.x,xxx.y-yyy.y);
    biu=biu/2.0;
    MyPoint res=yyy+biu;
    return res;

}

void ShowAndQuit(Mat& rst2,string info="test")
{
      while(1)
    {
        namedWindow(info, 0);
        imshow(info,rst2);
        if(waitKey(30)>=0)
        break;
    }
    destroyWindow(info);
}
void  solve_red(string lujing,bool if_blue)
{
    
    Mat image=imread(lujing.c_str());
    
    Mat ti;
    //转化为HSV
    cvtColor(image,ti,COLOR_BGR2HSV);   
    Mat rst;
    //调整阈值
    if(if_blue)
    adjust_to_filter_blue();
    else
    adjust_to_filter_red();
    //二值化
    inRange(ti,Scalar(minh,mins,minv),Scalar(maxh,maxs,maxv),rst);      
    ShowAndQuit(rst,"二值化");
    //寻找轮廓   
    vector<vector<Point> >ctr;
    vector<Vec4i> hir;
    findContours(rst,ctr,hir,CV_RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
    vector<RotatedRect> rects;
    vector<RotatedRect> shai_1;
    Mat rst2;
    Mat rst3;
    Mat rst4;
    //为了展示过程，复制几份副本
    cvtColor(rst,rst2,COLOR_GRAY2RGB);

    cvtColor(rst,rst3,COLOR_GRAY2RGB);

    cvtColor(rst,rst4,COLOR_GRAY2RGB);

    //便历轮廓，同时画框框
    for(int i=0;i<ctr.size();i++)
    {
        rects.push_back(minAreaRect(ctr[i]));
        Point2f vers[4];
        rects[i].points(vers);
        double mianji=rects[i].size.width*rects[i].size.height;
        //StrokeRectOnMat(rst,Scalar(0,255,0),rects[i],2);
         for(int j=0;j<4;j++)
             {
            line(rst4,vers[j],vers[(j+1)%4],Scalar(0,255,0),2);       
             }
        //如果筛选出是灯条
        if(ifDengtiao(rects[i]))
        {
            shai_1.push_back(rects[i]);
            //存储
           for(int j=0;j<4;j++)
             {
            line(rst2,vers[j],vers[(j+1)%4],Scalar(0,255,0),2);       
             }
             //框起来
        }
    }
    ShowAndQuit(rst4,"筛选轮廓");

    ShowAndQuit(rst2,"筛选灯条");

    int tp=0;
    //below are pairing part
    int lim=shai_1.size();
    for (int i = 0; i < lim; i++)
    {
        for(int j=i+1;j<lim;j++)
        {
            //配对
            if(pairing(shai_1[i],shai_1[j]))
            {
                
                StrokeRectOnMat(rst3,clrs[tp],shai_1[i],2);
                StrokeRectOnMat(rst3,clrs[tp],shai_1[j],2);
                tp++;
                MyPoint ct1(shai_1[i].center.x,shai_1[i].center.y);
                MyPoint ct2(shai_1[j].center.x,shai_1[j].center.y);
                MyPoint cnter=middlePoint(ct1,ct2);
                Point p2(cnter.x,cnter.y);
                circle(rst3, p2, 20,Scalar(102,204,255),-1);
            }
        }
       
        /* code */
    }
    ShowAndQuit(rst3,"装甲板中心和灯条配对");
    imwrite("armor.jpg",rst3);
   }

   int main()
   {
       cout<<"输入0代表找红灯，输入1代表找蓝灯\n";
       int input;
       cin>>input;
       solve_red("test.jpg",input==1);
       return 0;
   }
