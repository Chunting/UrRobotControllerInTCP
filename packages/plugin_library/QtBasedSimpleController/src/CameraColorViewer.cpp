//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <highgui.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_global_object_factory.h>
#include "CameraColorViewer.h"


CameraColorViewer::CameraColorViewer()
        : QObject(nullptr){
    m_captureTimer = new QTimer(this);
    m_captureTimer->setInterval(1);
    connect(m_captureTimer, &QTimer::timeout, this, &CameraColorViewer::updateCamera);
}

CameraColorViewer::~CameraColorViewer(){
    stop();
}

bool CameraColorViewer::start(){
    if (m_camera && m_captureTimer) {
        if (m_camera->open()) {
            m_captureTimer->start();
            return true;
        }
    }
    return false;
}

void CameraColorViewer::pause(){
    if (m_captureTimer)
        m_captureTimer->stop();
}

void CameraColorViewer::stop(){
    pause();
    if (m_camera) {
        m_camera->close();
    }
}

bool CameraColorViewer::setup(const std::string& xmlConfigFilePath){
    auto pFactory = cobotsys::GlobalObjectFactory::instance();
    if (pFactory) {
        auto pObject = pFactory->createObject("Kinect2CameraFactory, Ver 1.0", "Kinect2");
        m_camera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(pObject);
        if (m_camera) {
            auto shObj = shared_from_this();
            auto shObs = std::dynamic_pointer_cast<cobotsys::CameraStreamObserver>(shObj);
            m_camera->attach(shObs);
        }
        return true;
    }
    return false;
}

void CameraColorViewer::onCameraStreamUpdate(const std::vector<cobotsys::CameraStreamObserver::StreamFrame>& frames){
    for (const auto& frame: frames) {
        if (frame.frame.type == cobotsys::CameraFrameType::Color) {
            cv::Mat m;
            cv::pyrDown(frame.frame.image, m);
            cv::imshow(frame.frame.typeName(), m);

            char key = (char) cv::waitKey(5);
            if (key == 27) {
                QCoreApplication::quit();
            }
        }
    }
}

void CameraColorViewer::updateCamera(){
    if (m_camera) {
        m_camera->capture();
    }
}
