//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include "KinectCameraDetector.h"


class KinectCameraDetector::KinectCameraDetectorImpl {
public:
    libfreenect2::Freenect2 freenect2;

    KinectCameraDetectorImpl(){
        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    }
};


KinectCameraDetector::KinectCameraDetector(QObject* parent)
        : QObject(parent), m_impl(new KinectCameraDetectorImpl){

    m_timer = new QTimer(this);
    m_timer->setInterval(200);
    connect(m_timer, &QTimer::timeout, this, &KinectCameraDetector::checkCameraDeviceNum);

    m_dev_num = -1;
}

KinectCameraDetector::~KinectCameraDetector(){
}

void KinectCameraDetector::checkCameraDeviceNum(){
    int num_dev = m_impl->freenect2.enumerateDevices();

    if (num_dev > 0) {

        if (m_dev_num != num_dev) {
            Q_EMIT cameraFound(true);
        }
    } else {
        if (m_dev_num != num_dev) {
            Q_EMIT cameraFound(false);
        }
    }
    m_dev_num = num_dev;
}

void KinectCameraDetector::startCheck(){
    m_timer->start();
}
