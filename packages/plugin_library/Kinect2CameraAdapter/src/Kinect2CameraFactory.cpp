//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <libfreenect2/logger.h>
#include "Kinect2CameraFactory.h"
#include "Kinect2Camera.h"
#include "CameraPreview.h"

Kinect2CameraFactory::Kinect2CameraFactory(){
//    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
}

Kinect2CameraFactory::~Kinect2CameraFactory(){
}

std::vector<std::string> Kinect2CameraFactory::getSupportTypes(){
    return {"Kinect2", "CameraPreview"};
}

std::string Kinect2CameraFactory::getFactoryType(){
    return "Kinect2CameraFactory, Ver 1.0";
}

std::shared_ptr<cobotsys::AbstractObject> Kinect2CameraFactory::createObject(const std::string& type){
    if (type == "Kinect2")
        return std::make_shared<Kinect2Camera>();
    if (type == "CameraPreview")
        return std::make_shared<CameraPreview>();
    return nullptr;
}
