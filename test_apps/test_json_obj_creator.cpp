//
// Created by 潘绪洋 on 17-3-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_camera.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cobotsys_file_finder.h>
#include <extra2.h>
#include <opencv2/highgui.hpp>

void loop(cobotsys::ObjectGroup& objectGroup){
    auto pCamera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(objectGroup.getObject("camera"));
    auto pViewer = std::dynamic_pointer_cast<cobotsys::CameraStreamObserver>(objectGroup.getObject("view"));

    pCamera->attach(pViewer);
    if (pCamera->open()) {
        char qkey = 0;
        while (qkey != 27) {
            pCamera->capture();
            qkey = cv::waitKey(1);
        }
    }
}

int main(int argc, char** argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib/plugins");

    QJsonObject jsonObject;
    if (loadJson(jsonObject, std::string("CameraView.json"))) {
        cobotsys::ObjectGroup objectGroup;
        if (objectGroup.init(jsonObject)) {
            loop(objectGroup);
        }
    }
    return 0;
}