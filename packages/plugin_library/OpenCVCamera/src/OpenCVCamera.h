//
// Created by 潘绪洋 on 17-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_OPENCVCAMERA_H
#define COBOTSYS_OPENCVCAMERA_H

#include <cobotsys_abstract_camera.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cobotsys;
using namespace cv;

class OpenCVCamera: public AbstractCamera {
public:
    OpenCVCamera();
    virtual ~OpenCVCamera();

    virtual bool setup(const QString& configFilePath);

    virtual bool open(int deviceId = 0);
    virtual void close();
    virtual void attach(const shared_ptr<CameraStreamObserver>& observer);
    virtual bool capture(int waitMs = -1); /// @note 控制相机进行一次图像捕获

    virtual std::string getManufacturer() const;
    virtual std::string getFullDescription() const;
    virtual std::string getSerialNumber() const;

    virtual int getImageWidth(int imageIdx = 0) const;
    virtual int getImageHeight(int imageIdx = 0) const;
    virtual ImageType getImageType(int imageIdx = 0) const;
    virtual int getImageCount() const;

    virtual void clearAttachedObject();
protected:
    void notify();
protected:
    std::vector<shared_ptr<CameraStreamObserver> > m_observers;
    std::mutex m_mutex;
    VideoCapture m_videoCapture;
    Mat m_mat;
};


#endif //COBOTSYS_OPENCVCAMERA_H
