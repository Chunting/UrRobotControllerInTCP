//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_camera.h>

int main(int argc, char** argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);


    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib");

    auto pObject = globalObjectFactory.createObject("Kinect2CameraFactory, Ver 1.0", "Kinect2");

    std::shared_ptr<cobotsys::AbstractCamera> pCamera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(pObject);
    pCamera->open();
    return 0;
}