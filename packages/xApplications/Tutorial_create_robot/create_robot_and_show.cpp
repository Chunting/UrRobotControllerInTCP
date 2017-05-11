//
// Created by 潘绪洋 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <cobotsys_abstract_camera.h>
#include <cobotsys_file_finder.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>
#include <QFileDialog>

int main(int argc, char** argv) {
    QApplication a(argc, argv);

    // 初始化cobotsys内部的变量
    cobotsys::init_library(argc, argv);

    // cobotsys的对象工厂
    cobotsys::GlobalObjectFactory globalObjectFactory;

    // 加载默认路径下的所有插件
    globalObjectFactory.loadLibrarys();

    auto pObject = globalObjectFactory.createObject("URRealTimeDriverFactory, Ver 1.0", "URRealTimeDriver");
    auto pFilter = globalObjectFactory.createObject("TutorialPlugin_a, Ver 1.0", "EmptyRobotJointMoveFilter");

    auto pRobot = std::dynamic_pointer_cast<cobotsys::AbstractArmRobotRealTimeDriver>(pObject);

    if (pRobot && pFilter) {
        if (pObject->setup("CONFIG/UrRobotConfig/ur_sim_config.json")) {
            pRobot->setTargetJointFilter(std::dynamic_pointer_cast<cobotsys::ArmRobotJointTargetFilter>(pFilter));
            pRobot->start();
            return a.exec();
        }
    }
    return 0;
}