//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys.h>
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <cobotsys_abstract_camera.h>
#include <cobotsys_file_finder.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>

bool loop(cobotsys::ObjectGroup& objectGroup){
    auto pWidget = std::dynamic_pointer_cast<cobotsys::AbstractControllerWidget>(objectGroup.getObject("Widget"));
    if (pWidget) {
        pWidget->show();
        return true;
    }

    return false;
}

int main(int argc, char** argv){
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    if (argc <= 1) {
        COBOT_LOG.info() << "No json file name!";
        return 1;
    }

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib/plugins");

    QJsonObject jsonObject;

    if (loadJson(jsonObject, std::string(argv[1]))) {
        cobotsys::ObjectGroup objectGroup;
        if (objectGroup.init(jsonObject)) {
            if (loop(objectGroup)) {
                auto r = a.exec();
                return r;
            }
        }
    }
    return 0;
}