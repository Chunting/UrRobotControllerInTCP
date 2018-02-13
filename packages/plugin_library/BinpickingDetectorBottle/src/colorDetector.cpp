//
// Created by vimjian on 17-4-25.
//

#include "colorDetector.h"

//设置色彩距离阈值，阈值必须是正的，否则设为0
void ColorDetector::setColorDistanceThreshold(int distance)
{
    if (distance < 0)
        distance = 0;
    minDist = distance;
}

//获取色彩距离阈值
int ColorDetector::getColorDistanceThreshold() const
{
    return minDist;
}

//设置需检测的颜色
void ColorDetector::setTargetColor(unsigned char red, unsigned char green, unsigned char blue)
{
//BGR顺序
    target[2] = red;
    target[1] = green;
    target[0] = blue;

//hsv
    Mat tmp(1,1,CV_8UC3);
    tmp.at<Vec3b>(0,0)=target;
    cvtColor(tmp,tmp,CV_BGR2HSV);
    targetHSV=tmp.at<Vec3b>(0,0);
}


//设置需检测的颜色
void ColorDetector::setTargetColor(cv::Vec3b color)
{
    //rgb
    target = color;
    //hsv
    Mat tmp(1,1,CV_8UC3);
    tmp.at<Vec3b>(0,0)=target;
    cvtColor(tmp,tmp,CV_RGB2HSV);
    targetHSV=tmp.at<Vec3b>(0,0);
}

//获取需检测的颜色
cv::Vec3b ColorDetector::getTargetColor() const
{
    return target;
}

cv::Mat ColorDetector::process(const cv::Mat &image)//核心的处理方法
{
//按需重新分配二值图像
//与输入图像的尺寸相同，但是只有一个通道
    result.create(image.rows, image.cols, CV_8U);

//得到迭代器
    cv::Mat_<cv::Vec3b>::const_iterator it = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = image.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = result.begin<uchar>();
    for (; it != itend; ++it, ++itout)//处理每个像素
    {
//计算离目标颜色的距离
        if (getDistance(*it) < minDist)
        {
            *itout = 255;
        } else
        {
            *itout = 0;
        }
    }
    return result;
}

cv::Mat ColorDetector::processHSV(const cv::Mat &image)
{
    //按需重新分配二值图像
//与输入图像的尺寸相同，但是只有一个通道
    result.create(image.rows, image.cols, CV_8U);
    cv::Mat hsvMat;
    //转换颜色通道
    cvtColor(image,hsvMat,CV_BGR2HSV);

//得到迭代器
    cv::Mat_<cv::Vec3b>::const_iterator it = hsvMat.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = hsvMat.end<cv::Vec3b>();
    cv::Mat_<uchar>::iterator itout = result.begin<uchar>();
    for (; it != itend; ++it, ++itout)//处理每个像素
    {
//计算离目标颜色的距离
        if (getDistanceHSV(*it) < minDist && ((*it)[1]>43)  && ((*it)[2]>43))
        {
            *itout = 255;
        } else
        {
            *itout = 0;
        }
    }
    return result;
}

void ColorDetector::setTargetHSV(cv::Vec3b hsv)
{
    targetHSV=hsv;
}
