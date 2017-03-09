//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "Kinect2CameraFactory.h"

std::shared_ptr<Kinect2CameraFactory> kinect2CameraFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (kinect2CameraFactory == nullptr) {
        kinect2CameraFactory = std::make_shared<Kinect2CameraFactory>();
    }

    return kinect2CameraFactory.get();
};