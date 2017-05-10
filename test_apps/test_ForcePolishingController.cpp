//
// Created by 潘绪洋 on 17-3-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_controller.h>
#include <QApplication>

int main(int argc, char** argv){
    QApplication a(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;

    std::shared_ptr<cobotsys::AbstractObject> pObject;

    cobotsys::init_library(argc, argv);

    globalObjectFactory.loadLibrarys();

    pObject = globalObjectFactory.createObject("ForcePolishingController, Ver 1.0", "ForcePolishingController");

    auto pController = std::dynamic_pointer_cast<cobotsys::AbstractController>(pObject);

	QString ss;
	pController->setup(ss);

    pController->start();

    return a.exec();
}
