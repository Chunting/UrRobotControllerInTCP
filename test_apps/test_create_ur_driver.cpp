//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_camera.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>
#include <cobotsys_abstract_robot_driver.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv){
    QApplication a(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;

    std::shared_ptr<cobotsys::AbstractObject> pObject;

    cobotsys::init_library(argc, argv);

    globalObjectFactory.loadLibrarys("../lib");

    pObject = globalObjectFactory.createObject("UrRobotDriverFactory, Ver 1.0", "UrAdapter");
    auto pController = std::dynamic_pointer_cast<cobotsys::AbstractRobotDriver>(pObject);

    pObject = globalObjectFactory.createObject("UrRobotDriverFactory, Ver 1.0", "UrStatusDebugger");
    auto pDebugger = std::dynamic_pointer_cast<cobotsys::RobotStatusObserver>(pObject);

    if (pController) {
        pController->attach(pDebugger);
        pController->setup("");
        pController->start();

        std::this_thread::sleep_for(std::chrono::microseconds(500));
//        while(1){
//            pController->move(0, cv::Point3d(), cv::Vec3d());
//            std::this_thread::sleep_for(std::chrono::microseconds(15));
//        }
//        pController->move(0, {0, 0, 0, 0, 0, 0});
//        pController->move(0, cv::Point3d(), cv::Vec3d());
//        pController->move(0, cv::Point3d(), cv::Vec3d());
//        pController->move(0, cv::Point3d(), cv::Vec3d());
//        pController->move(0, cv::Point3d(), cv::Vec3d());
//        pController->move(0, cv::Point3d(), cv::Vec3d());
//        pController->move(0, cv::Point3d(), cv::Vec3d());
    }

    return a.exec();
}
