//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_CAMERA_H
#define PROJECT_COBOTSYS_ABSTRACT_CAMERA_H


#include "cobotsys_abstract_object.h"
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "cobotsys_data_types.h"


namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 */

/**
 * @defgroup camera
 * @brief 相机API接口
 * @{
 */

class CameraStreamObserver {
public:
    CameraStreamObserver();
    virtual ~CameraStreamObserver();
    virtual void onCameraStreamUpdate(const CameraFrame& cameraFrame) = 0;
};

class AbstractCamera : public AbstractObject {
public:
    AbstractCamera();
    virtual ~AbstractCamera();

    virtual bool open(int deviceId = 0) = 0;
    virtual void close() = 0;
    virtual void attach(std::shared_ptr<CameraStreamObserver> observer) = 0;
    virtual bool capture(int waitMs = -1) = 0; /// @note 控制相机进行一次图像捕获

    virtual std::string getManufacturer() const = 0;
    virtual std::string getFullDescription() const = 0;
    virtual std::string getSerialNumber() const = 0;

    virtual int getImageWidth(int imageIdx = 0) const = 0;
    virtual int getImageHeight(int imageIdx = 0) const = 0;
    virtual ImageType getImageType(int imageIdx = 0) const = 0;
    virtual int getImageCount() const = 0;
};

/**
 * @}
 * @}
 */
}


#endif //PROJECT_COBOTSYS_ABSTRACT_CAMERA_H
