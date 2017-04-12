//
// Created by 杨帆 on 17-4-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_global_object_factory.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_abstract_kinematic_solver.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>
#include <cobotsys_file_finder.h>
int main(int argc, char** argv){
    QApplication a(argc, argv);
	cobotsys::init_library(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;

    std::shared_ptr<cobotsys::AbstractObject> pObject;

    cobotsys::init_library(argc, argv);

    globalObjectFactory.loadLibrarys();

    pObject = globalObjectFactory.createObject("KinematicSolverFactory, Ver 1.0", "KinematicSolver");

    auto pKSolver = std::dynamic_pointer_cast<cobotsys::AbstractKinematicSolver>(pObject);
	//COBOT_LOG.info() << QString("configFilePath") << cobotsys::FileFinder::find("CONFIG/force_control/kinematic_solver_config.json");
	//pKSolver->setup("CONFIG/force_control/kinematic_solver_config_no_comment.json");
	pKSolver->setup("CONFIG/force_control/kinematic_solver_ur3_config.json");
    return a.exec();
}