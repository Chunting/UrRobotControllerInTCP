//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_KINECT2CAMERA_H
#define PROJECT_KINECT2CAMERA_H


#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_camera.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <mutex>
#include <cobotsys_logger.h>
#include <chrono>


class Kinect2Camera : public cobotsys::AbstractCamera {
public:
    Kinect2Camera();
    virtual ~Kinect2Camera();

    const cobotsys::CameraInformation& getCameraInformation() const;

    virtual bool open(int deviceId = 0);
    virtual void close(); ///  @note 最好不要在回调函数里调用close函数。
    virtual void attach(std::shared_ptr<cobotsys::CameraStreamObserver> observer);

    virtual bool capture(int waitMs); /// @note 控制相机进行一次图像捕获

protected:
    libfreenect2::PacketPipeline* createPipeline(int deviceId);
    void notify(const std::vector<cobotsys::CameraStreamObserver::StreamFrame>& frames);

    void delayClose();
protected:
    cobotsys::CameraInformation m_cameraInfo;

    std::vector<std::shared_ptr<cobotsys::CameraStreamObserver> > m_observers;

    libfreenect2::Freenect2* m_freenect2;
    libfreenect2::Freenect2Device* m_freenectDev;
    int m_devicdId;

    std::string m_deviceSerialNumber;

    std::shared_ptr<libfreenect2::SyncMultiFrameListener> m_listener;

    std::mutex m_ioMutex;

    bool m_isNotifyCalling;
    bool m_isCloseCallInNotify;

    bool m_isOpened;
};


#endif //PROJECT_KINECT2CAMERA_H
