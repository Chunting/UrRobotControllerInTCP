//
// Created by 潘绪洋 on 17-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "OpenCVCamera.h"

OpenCVCamera::OpenCVCamera() {
}

OpenCVCamera::~OpenCVCamera() {
}

bool OpenCVCamera::setup(const QString& configFilePath) {
    COBOT_LOG.message() << "Use default config from usb camera.";
    return true;
}

bool OpenCVCamera::open(int deviceId) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    if (m_videoCapture.open(deviceId)) {
        return true;
    }
    COBOT_LOG.warning() << "Fail to open camera: " << deviceId;
    return false;
}

void OpenCVCamera::close() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_videoCapture.release();
}

void OpenCVCamera::attach(const std::shared_ptr<CameraStreamObserver>& observer) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (observer) {
        for (auto& iter : m_observers) {
            if (observer == iter) {
                return;
            }
        }
        m_observers.push_back(observer);
    }
}

bool OpenCVCamera::capture(int waitMs) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    if (m_videoCapture.isOpened()) {
        if (m_videoCapture.grab()) {
            if (m_videoCapture.retrieve(m_mat)) {
                notify();
                return true;
            }
        }
    }
    return false;
}

std::string OpenCVCamera::getManufacturer() const {
    return std::string();
}

std::string OpenCVCamera::getFullDescription() const {
    return std::string();
}

std::string OpenCVCamera::getSerialNumber() const {
    return std::string();
}

int OpenCVCamera::getImageWidth(int imageIdx) const {
    return m_mat.cols;
}

int OpenCVCamera::getImageHeight(int imageIdx) const {
    return m_mat.rows;
}

ImageType OpenCVCamera::getImageType(int imageIdx) const {
    return ImageType::Color;
}

int OpenCVCamera::getImageCount() const {
    return 1;
}

void OpenCVCamera::notify() {
    CameraFrame frame;
    frame.capture_time = std::chrono::high_resolution_clock::now();
    if (m_mat.channels() == 3)
        frame.frames.push_back({ImageType::Color, m_mat});
    else
        frame.frames.push_back({ImageType::Mono, m_mat});

    for (auto& iter :  m_observers) {
        iter->onCameraStreamUpdate(frame, this);
    }
}

void OpenCVCamera::clearAttachedObject() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_observers.clear();
}


bool OpenCVCamera::isOpened() const {
    return m_videoCapture.isOpened();
}
