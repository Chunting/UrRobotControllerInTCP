//
// Created by 潘绪洋 on 17-1-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "Kinect2CameraWrap.h"
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include "cobotsys_logger.h"

using namespace std;

class CameraKinect2::CameraKinect2Priv {
public:
    libfreenect2::Freenect2* freenect2;
    libfreenect2::Freenect2Device* dev;
    std::shared_ptr<libfreenect2::PacketPipeline> pipeline;
    int deviceId;
    std::string serial;
    int types;

    std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener;
    libfreenect2::FrameMap frames;

    std::shared_ptr<libfreenect2::Registration> registration;
    std::shared_ptr<libfreenect2::Frame> undistorted;
    std::shared_ptr<libfreenect2::Frame> registered;

    bool loopCamera;
    bool isInitSuccess;

    std::function<void(const CameraKinect2::IMAGE&)> imageHandler;
    CameraKinect2::IMAGE image;

    CameraKinect2Priv(){
        freenect2 = new libfreenect2::Freenect2;
        isInitSuccess = false;
        deviceId = -1;
        loopCamera = false;
        dev = nullptr;
        pipeline = nullptr;
        types = libfreenect2::Frame::Ir | libfreenect2::Frame::Depth | libfreenect2::Frame::Color;
    }

    bool createPacketPipeline(){
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        if (pipeline == nullptr)
            pipeline = std::make_shared<libfreenect2::CudaPacketPipeline>(deviceId);
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if (pipeline == nullptr)
            pipeline = std::make_shared<libfreenect2::OpenCLPacketPipeline>(deviceId);
#endif
        if (pipeline == nullptr)
            pipeline = std::make_shared<libfreenect2::OpenGLPacketPipeline>();
        if (pipeline == nullptr)
            pipeline = std::make_shared<libfreenect2::CpuPacketPipeline>();

        if (pipeline == nullptr) {
            std::cerr << "Error, create pipeline fail!" << endl;
            return false;
        }
        return true;
    }

    bool discoveryCameraDevice(){
        if (freenect2->enumerateDevices() == 0)
            return false;

        serial = freenect2->getDefaultDeviceSerialNumber();
        return true;
    }

    bool openDevice(){
        if (pipeline)
            dev = freenect2->openDevice(serial, pipeline.get());
        else
            dev = freenect2->openDevice(serial);

        if (dev == nullptr)
            return false;
        return true;
    }

    void setupListener(){
        listener = std::make_shared<libfreenect2::SyncMultiFrameListener>(types);
        if (dev) {
            dev->setColorFrameListener(listener.get());
            dev->setIrAndDepthFrameListener(listener.get());
        }
    }

    bool startCapture(){
        if (dev->start()) {
            cout << "device serial  : " << dev->getSerialNumber() << endl;
            cout << "device firmware: " << dev->getFirmwareVersion() << endl;
            return true;
        }
        return false;
    }

    void createDefaultRegistration(){

        auto irParam = dev->getIrCameraParams();
        auto crParam = dev->getColorCameraParams();

        cobotsys::cout_formater fmt;
        fmt.section("IR Camera").printArray<float>(&irParam.fx, sizeof(irParam) / 4).newline();
        fmt.section("Color Camera").printArray<float>(&crParam.fx, sizeof(crParam) / 4).newline();


        registration = std::make_shared<libfreenect2::Registration>(dev->getIrCameraParams(),
                                                                    dev->getColorCameraParams());
        undistorted = std::make_shared<libfreenect2::Frame>(512, 424, 4);
        registered = std::make_shared<libfreenect2::Frame>(512, 424, 4);
    }

    void finalClose(){
        cout << "Close Camera Device" << endl;
        if (dev) {
            dev->stop();
            dev->close();
        }
        isInitSuccess = false;
    }

    void loop(){
        cv::Mat cv_color;
        cv::Mat cv_depth;
        while (loopCamera) {
            loopOnce(10 * 1000);
        }

        finalClose();
    }

    void close(){
        if (loopCamera) {
            loopCamera = false;
            return;
        }

        finalClose();
    }

    void loopOnce(int waitMax = 1){
        cv::Mat cv_color;
        cv::Mat cv_depth;
        if (listener->hasNewFrame()) {
            if (listener->waitForNewFrame(frames, waitMax)) {
                libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
                libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
                libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

                if (registration)
                    registration->apply(rgb, depth, undistorted.get(), registered.get());

                cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(image.raw_color);
                cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(image.raw_ir);
                cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(image.raw_depth);
                cv::Mat(undistorted->height, undistorted->width, CV_32FC1, undistorted->data).copyTo(
                        image.undistorted_depth);
                cv::Mat(registered->height, registered->width, CV_32FC1, registered->data).copyTo(
                        image.registered_color);

                if (imageHandler)
                    imageHandler(image);
                listener->release(frames);
            }
        }
    }
};


CameraKinect2::CameraKinect2()
        : priv_(new CameraKinect2Priv){
}

CameraKinect2::~CameraKinect2(){
}

void CameraKinect2::loopCamera(){
    priv_->loopCamera = true;
    priv_->loop();
}

bool CameraKinect2::initCamera(){
    if (priv_->isInitSuccess)
        return true;

    if (!priv_->createPacketPipeline()) return false;
    if (!priv_->discoveryCameraDevice()) return false;
    if (!priv_->openDevice()) return false;

    priv_->setupListener();

    if (priv_->startCapture()) {
        priv_->createDefaultRegistration();
        priv_->isInitSuccess = true;
        return true;
    }
    return false;
}

void CameraKinect2::exitLoop(){
    priv_->loopCamera = false;
}

void CameraKinect2::regisiterImageCallback(std::function<void(const CameraKinect2::IMAGE&)> func){
    priv_->imageHandler = func;
}

void CameraKinect2::loopCameraOnce(){
    priv_->loopOnce();
}

void CameraKinect2::closeCamera(){
    COBOT_LOG.message() << "Closing Kinect2 Camera" << endl;
    priv_->close();
}


