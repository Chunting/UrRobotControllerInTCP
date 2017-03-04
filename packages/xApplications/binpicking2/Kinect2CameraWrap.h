//
// Created by 潘绪洋 on 17-1-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_CAMERA_KINECT2_H
#define PROJECT_COBOTSYS_CAMERA_KINECT2_H

#include <cobotsys.h>
#include <highgui.h>
#include <cv.h>
#include <fstream>
#include <functional>
#include <memory>


class CameraKinect2 {
public:
    struct IMAGE {
        cv::Mat raw_color;
        cv::Mat raw_ir;
        cv::Mat raw_depth;

        cv::Mat undistorted_depth;
        cv::Mat registered_color; // 512x424
        cv::Mat big_depth;
    };

public:
    CameraKinect2();
    ~CameraKinect2();

    bool initCamera();
    void closeCamera();
    void regisiterImageCallback(std::function<void(const IMAGE& image)> func);

    void loopCamera();
    void loopCameraOnce();
    void exitLoop();
private:
    class CameraKinect2Priv;
    std::shared_ptr<CameraKinect2Priv> priv_;
};


#endif //PROJECT_COBOTSYS_CAMERA_KINECT2_H
