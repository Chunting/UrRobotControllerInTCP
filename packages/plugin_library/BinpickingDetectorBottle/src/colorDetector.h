//
// Created by vimjian on 17-4-25.
//

#ifndef OPENNI_GRABBER_COLORDETECTOR_H
#define OPENNI_GRABBER_COLORDETECTOR_H


#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace cv;


    class ColorDetector
    {
    private:
//最小可接受距离
        int minDist;
//目标色
        cv::Vec3b target;
        //
        cv::Vec3b targetHSV;
//结果图像
        cv::Mat result;

//计算与目标颜色的距离
        int getDistance(cv::Vec3b color)
        {
            return sqrt((color[0] - target[0])*(color[0] - target[0])
                   +(color[1] - target[1]) *(color[1] - target[1])
                   +(color[2] - target[2])*(color[2] - target[2]));
           // return abs(color[0] - target[0]) + abs(color[1] - target[1]) + abs(color[2] - target[2]);
        }

    public:
//空构造函数
        ColorDetector() : minDist(100)
        {
//初始化默认参数
            target[0] = target[1] = target[2] = 0;
        }

        void setColorDistanceThreshold(int distance);

        int getColorDistanceThreshold() const;

        void setTargetColor(unsigned char red, unsigned char green, unsigned char blue);

        void setTargetColor(cv::Vec3b color);

        void setTargetHSV(cv::Vec3b hsv);

        int  getDistanceHSV(cv::Vec3b hsv)
        {
            return sqrt((hsv[0] - targetHSV[0])*(hsv[0] - targetHSV[0]));
        }
        cv::Vec3b getTargetColor() const;

        cv::Mat process(const cv::Mat &image);
        cv::Mat processHSV(const cv::Mat &image);
    };



#endif //OPENNI_GRABBER_COLORDETECTOR_H
