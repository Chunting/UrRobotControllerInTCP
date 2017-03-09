//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "Kinect2Camera.h"


Kinect2Camera::Kinect2Camera(){
    m_freenect2 = new libfreenect2::Freenect2;
    m_pipeline = nullptr;
    m_freenectDev = nullptr;
    m_devicdId = -1;
}

Kinect2Camera::~Kinect2Camera(){
    close();
    delete m_freenect2;
}

const cobotsys::CameraInformation& Kinect2Camera::getCameraInformation() const{
    return m_cameraInfo;
}

bool Kinect2Camera::open(int deviceId){
    close();

    std::lock_guard<std::mutex> lock(m_ioMutex);

    int numDevice = m_freenect2->enumerateDevices();

    if (numDevice == 0) {
        COBOT_LOG.warning() << "No Device is in system";
        return false;
    }

    m_deviceSerialNumber = m_freenect2->getDeviceSerialNumber(deviceId);
    createPipeline(deviceId);

    /// @note Open Device, use pipeline
    if (m_pipeline) {
        m_freenectDev = m_freenect2->openDevice(m_deviceSerialNumber, m_pipeline);
    } else {
        COBOT_LOG.warning() << "Can not create pipeline";
        m_freenectDev = m_freenect2->openDevice(m_deviceSerialNumber);
    }

    if (m_freenectDev == nullptr) {
        COBOT_LOG.error() << "Can not create kinect2 device";
        return false;
    }

    /// @note Listener(Observer) frame
    int types = libfreenect2::Frame::Ir | libfreenect2::Frame::Depth | libfreenect2::Frame::Color;
    m_listener = std::make_shared<libfreenect2::SyncMultiFrameListener>(types);
    m_freenectDev->setColorFrameListener(m_listener.get());
    m_freenectDev->setIrAndDepthFrameListener(m_listener.get());

    /// @note Start Device Stream.
    if (m_freenectDev->start()) {
        COBOT_LOG.info() << "device serial  : " << m_freenectDev->getSerialNumber() << endl;
        COBOT_LOG.info() << "device firmware: " << m_freenectDev->getFirmwareVersion() << endl;
        return true;
    }
    return false;
}

void Kinect2Camera::close(){
    std::lock_guard<std::mutex> lock(m_ioMutex);

    if (m_freenectDev) {
        m_freenectDev->stop();
        m_freenectDev->close();
        m_freenectDev = nullptr;
    }

    m_listener = nullptr;

    if (m_pipeline) {
        delete m_pipeline;
        m_pipeline = nullptr;
    }

    m_deviceSerialNumber.clear();
    m_devicdId = -1;
}

void Kinect2Camera::attach(std::shared_ptr<cobotsys::CameraStreamObserver> observer){
    for (auto& o : m_observers) {
        if (o == observer)
            return;;
    }

    m_observers.push_back(observer);
}

void Kinect2Camera::capture(){
    if (m_listener) {
        if (m_listener->hasNewFrame()) {
            libfreenect2::FrameMap frames;
            auto timeBeforeWait = std::chrono::high_resolution_clock::now();
            m_listener->waitForNewFrame(frames);
            auto timeAfterWait = std::chrono::high_resolution_clock::now();

            libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

            cv::Mat raw_color;
            cv::Mat raw_depth;
            cv::Mat raw_ir;

            cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(raw_color);
            cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(raw_ir);
            cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(raw_depth);

            cobotsys::CameraFrame c_color = {cobotsys::CameraFrameType::Color, raw_color};
            cobotsys::CameraFrame c_depth = {cobotsys::CameraFrameType::Depth, raw_depth};
            cobotsys::CameraFrame c_ir = {cobotsys::CameraFrameType::Ir, raw_ir};

            std::vector<cobotsys::CameraStreamObserver::StreamFrame> streamFrames;
            streamFrames.push_back({0, timeAfterWait, c_color});
            streamFrames.push_back({1, timeAfterWait, c_depth});
            streamFrames.push_back({2, timeAfterWait, c_ir});
            notify(streamFrames);

            m_listener->release(frames);
        }
    }
}


void Kinect2Camera::notify(const std::vector<cobotsys::CameraStreamObserver::StreamFrame>& frames){
    for (auto& observer : m_observers) {
        observer->onCameraStreamUpdate(frames);
    }
}

void Kinect2Camera::createPipeline(int deviceId){
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    if (m_pipeline == nullptr)
        m_pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
#endif

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    if (m_pipeline == nullptr)
        m_pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
#endif

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    if (m_pipeline == nullptr)
        m_pipeline = new libfreenect2::OpenGLPacketPipeline();
#endif

    if (m_pipeline == nullptr)
        m_pipeline = new libfreenect2::CpuPacketPipeline();
}


