//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_camera.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

int main(int argc, char** argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib/plugins");

    std::shared_ptr<cobotsys::AbstractObject> pObject;

    pObject = globalObjectFactory.createObject("Kinect2CameraFactory, Ver 1.0", "Kinect2");
    auto pCamera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(pObject);

    pObject = globalObjectFactory.createObject("Kinect2CameraFactory, Ver 1.0", "CameraPreview");
    auto pViewer = std::dynamic_pointer_cast<cobotsys::CameraStreamObserver>(pObject);

    if (pViewer == nullptr)
        return 1;

    pCamera->attach(pViewer);
    if (pCamera->open()) {
        char qkey = 0;
        while (qkey != 27) {
            pCamera->capture();
            qkey = cv::waitKey(1);
        }
    }
    return 0;
}
