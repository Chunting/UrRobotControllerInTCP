//
// Created by eleven on 17-4-19.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_camera.h>
#include <opencv/cv.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>
#include "CameraCalibrationWidget.h"

int main(int argc, char** argv){
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();

    std::shared_ptr<CameraCalibrationWidget> pController = std::make_shared<CameraCalibrationWidget>();
    QString ss;
    pController->setup(ss);
    pController->show();

    return a.exec();
}