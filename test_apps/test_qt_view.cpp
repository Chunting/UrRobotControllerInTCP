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
#include <cobotsys_abstract_controller.h>

int main(int argc, char** argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();

    std::shared_ptr<cobotsys::AbstractObject> pObject;

    pObject = globalObjectFactory.createObject("SimpleControllerFactory, Ver 1.0", "SimpleCameraView");
    auto pController = std::dynamic_pointer_cast<cobotsys::AbstractController>(pObject);

    if (pController) {
        if (pController->setup("")) {
            pController->start();
        }
    }
    return a.exec();
}
