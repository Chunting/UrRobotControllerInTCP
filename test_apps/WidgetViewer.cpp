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

bool loop(cobotsys::ObjectGroup& objectGroup) {
    auto pObject = objectGroup.getObject("Widget");
    auto pWidget = std::dynamic_pointer_cast<QWidget>(pObject);
    if (pWidget) {
        pObject->setup("");
        pWidget->show();
        return true;
    }
    COBOT_LOG.info() << "No Widget Found in JSON config.";
    return false;
}


int main(int argc, char** argv) {
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::FileFinder::addSearchPath("UI");

    QString json_path;

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();


    json_path = cobotsys::FileFinder::find("simple_widget_viewer.json").c_str();

    if (json_path.isEmpty()){
        COBOT_LOG.error() << "Can not load WidgetViewer JSON config";
        return 2;
    }

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