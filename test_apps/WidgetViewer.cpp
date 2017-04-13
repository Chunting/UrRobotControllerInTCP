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

int main(int argc, char** argv) {
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::FileFinder::addSearchPath("UI");

    QString json_path;

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();


    auto pObject = globalObjectFactory.createObject("SimpleUiFactory, Ver 1.0", "SimpleWidgetViewer");
    auto pWidget = std::dynamic_pointer_cast<QWidget>(pObject);
    if (pWidget) {
        pObject->setup("");
        pWidget->show();
        return a.exec();
    }
    return 0;
}