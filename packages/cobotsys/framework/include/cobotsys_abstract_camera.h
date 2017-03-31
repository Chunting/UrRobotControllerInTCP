//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_CAMERA_H
#define PROJECT_COBOTSYS_ABSTRACT_CAMERA_H


#include "cobotsys_abstract_object.h"
#include <vector>
#include <chrono>
#include <opencv/cv.h>


namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 */

/**
 * @defgroup camera
 * @{
 */
class CameraStreamObserver;


enum class CameraFrameType {
    Color, /// @note 8bit x 3, BGR, opencv use BGR as default, horse's ass
    Depth, /// @note change depth to float?
    Gray, /// @note 8bit
    Ir, /// @note 红外
};


/// @note 一个相机设备里输出的单幅图像，如果有多个输出，放在不同的帧。
struct CameraFrame {
    CameraFrameType type;
    cv::Mat image;

    std::string typeName() const;
};

class CameraInformation {
public:
    CameraInformation();
    CameraInformation(const std::string& info);
    CameraInformation(const CameraInformation& other);


    std::string getManufacturer() const;
    std::string getFullDescription() const;
    std::string getSerialNumber() const;

    int getFrameWidth(int frameId = 0) const;
    int getFrameHeight(int frameId = 0) const;
    CameraFrameType getFrameType(int frameId = 0) const;
    int getFrameCount() const;
protected:
    class CameraInformationImpl;
    std::shared_ptr<CameraInformationImpl> m_impl;
};

class CameraStreamObserver {
public:
    struct StreamFrame {
        int frameId;
        std::chrono::high_resolution_clock::time_point frameCaptureTime;
        const CameraFrame& frame;
    };
public:
    CameraStreamObserver();
    virtual ~CameraStreamObserver();
    virtual void onCameraStreamUpdate(const std::vector<StreamFrame>& frames) = 0;
};

class AbstractCamera : public AbstractObject {
public:
    AbstractCamera();
    virtual ~AbstractCamera();

    virtual const CameraInformation& getCameraInformation() const = 0;

    virtual bool open(int deviceId = 0) = 0;
    virtual void close() = 0;
    virtual void attach(std::shared_ptr<CameraStreamObserver> observer) = 0;
    virtual bool capture(int waitMs = -1) = 0; /// @note 控制相机进行一次图像捕获
};

/**
 * @}
 */

/**
 * @}
 */
}


#endif //PROJECT_COBOTSYS_ABSTRACT_CAMERA_H
