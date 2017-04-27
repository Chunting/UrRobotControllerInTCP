//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "Kinect2Camera.h"


Kinect2Camera::Kinect2Camera()
        : undistorted(512, 424, 4), registered(512, 424, 4) {
    m_freenect2 = new libfreenect2::Freenect2;
    m_freenectDev = nullptr;
    m_devicdId = -1;

    m_isNotifyCalling = false;
    m_isCloseCallInNotify = false;
    m_isOpened = false;

    registration = nullptr;
}

Kinect2Camera::~Kinect2Camera() {
    close();
    delete m_freenect2;
}

bool Kinect2Camera::isOpened() const {
    return m_isOpened;
}

bool Kinect2Camera::open(int deviceId) {
    if (m_isOpened)
        return true;

    std::lock_guard<std::mutex> lock(m_ioMutex);

    int numDevice = m_freenect2->enumerateDevices();

    if (numDevice == 0) {
        COBOT_LOG.warning() << "No Device is in system";
        return false;
    }

    m_deviceSerialNumber = m_freenect2->getDeviceSerialNumber(deviceId);
    auto pipeline = createPipeline(deviceId);

    /// @note Open Device, use pipeline
    if (pipeline) {
        m_freenectDev = m_freenect2->openDevice(m_deviceSerialNumber, pipeline);
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

        registration = new libfreenect2::Registration(m_freenectDev->getIrCameraParams(),
                                                      m_freenectDev->getColorCameraParams());

        m_isOpened = true;
        return true;
    }
    return false;
}

void Kinect2Camera::close() {
    std::lock_guard<std::mutex> lock(m_ioMutex);

    if (m_isNotifyCalling) {
        m_isCloseCallInNotify = true;
        return;
    }

    if (m_freenectDev) {
        m_freenectDev->stop();
        m_freenectDev->close();
        m_freenectDev = nullptr;
    }

    if (registration) {
        delete registration;
    }


    m_listener = nullptr;

    m_deviceSerialNumber.clear();
    m_devicdId = -1;
    m_isOpened = false;
}

void Kinect2Camera::attach(const std::shared_ptr<cobotsys::CameraStreamObserver>& observer) {
    for (auto& o : m_observers) {
        if (o == observer)
            return;
    }

    if (observer) {
        m_observers.push_back(observer);
    }
}

bool Kinect2Camera::capture(int waitMs) {
    if (m_listener) {
        libfreenect2::FrameMap frames;
        auto timeBeforeWait = std::chrono::high_resolution_clock::now();
        if (m_listener->waitForNewFrame(frames, waitMs)) {
            auto timeAfterWait = std::chrono::high_resolution_clock::now();

            libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

            registration->apply(rgb, depth, &undistorted, &registered);

            cv::Mat raw_color;
            cv::Mat raw_depth;
            cv::Mat raw_ir;
            cv::Mat undistorted_depth;
            cv::Mat registered_color;

            cv::Mat((int) rgb->height, (int) rgb->width, CV_8UC4, rgb->data).copyTo(raw_color);
            cv::Mat((int) depth->height, (int) depth->width, CV_32FC1, depth->data).copyTo(raw_depth);
            cv::Mat((int) ir->height, (int) ir->width, CV_32FC1, ir->data).copyTo(raw_ir);
            cv::Mat((int) undistorted.height, (int) undistorted.width, CV_32FC1, undistorted.data).copyTo(
                    undistorted_depth);
            cv::Mat((int) registered.height, (int) registered.width, CV_8UC4, registered.data).copyTo(
                    registered_color);

            cobotsys::ImageFrame c_color = {cobotsys::ImageType::Color, raw_color};
            cobotsys::ImageFrame c_depth = {cobotsys::ImageType::Depth, raw_depth};
            cobotsys::ImageFrame c_ir = {cobotsys::ImageType::Ir, raw_ir};


            cobotsys::CameraFrame streamFrames;
            streamFrames.capture_time = timeAfterWait;
            streamFrames.frames.push_back(c_color);
            streamFrames.frames.push_back(c_depth);
            streamFrames.frames.push_back(c_ir);
            streamFrames.frames.push_back({cobotsys::ImageType::Depth, undistorted_depth});
            streamFrames.frames.push_back({cobotsys::ImageType::Color, registered_color});

            notify(streamFrames);

            m_listener->release(frames);
            delayClose();
            return true;
        }
    }
    return false;
}


void Kinect2Camera::delayClose() {
    if (m_isCloseCallInNotify) {
        close();
    }
    m_isCloseCallInNotify = false;
}


void Kinect2Camera::notify(const cobotsys::CameraFrame& cameraFrame) {
    m_isNotifyCalling = true;
    for (auto& observer : m_observers) {
        observer->onCameraStreamUpdate(cameraFrame, this);
    }
    m_isNotifyCalling = false;
}

libfreenect2::PacketPipeline* Kinect2Camera::createPipeline(int deviceId) {
    libfreenect2::PacketPipeline* pipeline = nullptr;
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    if (pipeline == nullptr) {
        pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
        COBOT_LOG.info() << "CudaPacketPipeline created: " << pipeline;
    }
#endif

#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    if (pipeline == nullptr) {
        pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
        COBOT_LOG.info() << "OpenCLPacketPipeline created: " << pipeline;
    }
#endif

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    if (pipeline == nullptr) {
        pipeline = new libfreenect2::OpenGLPacketPipeline();
        COBOT_LOG.info() << "OpenGLPacketPipeline created: " << pipeline;
    }
#endif

    if (pipeline == nullptr) {
        pipeline = new libfreenect2::CpuPacketPipeline();
        COBOT_LOG.info() << "CpuPacketPipeline created: " << pipeline;
    }

    return pipeline;
}

bool Kinect2Camera::setup(const QString& configFilePath) {
    COBOT_LOG.info() << "This Camera Driver is not finish yet.";
    return true;
}

std::string Kinect2Camera::getManufacturer() const {
    return "MicroSoft";
}

std::string Kinect2Camera::getFullDescription() const {
    return "Xbox Kinect 2.0";
}

std::string Kinect2Camera::getSerialNumber() const {
    return m_deviceSerialNumber;
}

int Kinect2Camera::getImageWidth(int imageIdx) const {
    if (imageIdx == 0) return 1920;
    if (imageIdx == 1) return 512;
    if (imageIdx == 2) return 512;
    return 0;
}

int Kinect2Camera::getImageHeight(int imageIdx) const {
    if (imageIdx == 0) return 1080;
    if (imageIdx == 1) return 424;
    if (imageIdx == 2) return 424;
    return 0;
}

ImageType Kinect2Camera::getImageType(int imageIdx) const {
    if (imageIdx == 0) return ImageType::Color;
    if (imageIdx == 1) return ImageType::Depth;
    if (imageIdx == 2) return ImageType::Ir;
    return ImageType::Color;
}

int Kinect2Camera::getImageCount() const {
    return 3;
}



