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
#include <QtWidgets/QFileDialog>

bool loop(cobotsys::ObjectGroup& objectGroup){
    auto pWidget = std::dynamic_pointer_cast<QWidget>(objectGroup.getObject("Widget"));
    if (pWidget) {
        pWidget->show();
        return true;
    }
    COBOT_LOG.info() << "No Widget Found in JSON config.";
    return false;
}


int main(int argc, char** argv){
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::FileFinder::addSearchPath("test_ui_json");

    QString json_path;

    if (argc <= 1) {
        json_path = QFileDialog::getOpenFileName(nullptr, QString(), "../../data");
        if (json_path.isEmpty())
            return 1;
    } else {
        json_path = argv[1];
    }

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib/plugins");
    globalObjectFactory.loadLibrarys("../../lib/plugins");

    QJsonObject jsonObject;

    if (loadJson(jsonObject, json_path)) {
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