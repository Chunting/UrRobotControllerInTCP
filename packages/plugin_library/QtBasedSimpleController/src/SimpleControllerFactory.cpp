//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SimpleControllerFactory.h"
#include "CameraColorViewer.h"
#include "CameraColorViewer2.h"

std::shared_ptr<SimpleControllerFactory> localFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (localFactory == nullptr) {
        localFactory = std::make_shared<SimpleControllerFactory>();
    }

    return localFactory.get();
};

SimpleControllerFactory::SimpleControllerFactory(){
}

SimpleControllerFactory::~SimpleControllerFactory(){
}

std::vector<std::string> SimpleControllerFactory::getSupportTypes(){
    return {"SimpleCameraView", "CameraViewWidget"};
}

std::string SimpleControllerFactory::getFactoryType(){
    return "SimpleControllerFactory, Ver 1.0";
}

std::shared_ptr<cobotsys::AbstractObject> SimpleControllerFactory::createObject(const std::string& type){
    if (type == "SimpleCameraView")
        return std::make_shared<CameraColorViewer>();
    if (type == "CameraViewWidget")
        return std::make_shared<CameraColorViewer2>();
    return std::shared_ptr<cobotsys::AbstractObject>();
}
