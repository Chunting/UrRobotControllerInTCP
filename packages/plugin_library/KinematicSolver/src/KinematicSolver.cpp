//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <extra2.h>
#include "KinematicSolver.h"
#include <cobotsys_file_finder.h>
using namespace cobotsys;
KinematicSolver::KinematicSolver(){

}

KinematicSolver::~KinematicSolver(){

}

bool KinematicSolver::solve(const vector<double>& cur, const vector<double>& target, vector<double>& result) {
	return true;
}
bool KinematicSolver::setup(const QString& configFilePath) {
    //auto a = FileFinder::find("CONFI/force_control/ur3.urdf.xacro")
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_defaultSolverInfo = json["kinematic_solver_factory"].toString();
        auto file = cobotsys::FileFinder::find(json["robot_model_path"].toString().toStdString());
        m_urdf_path= QString().fromStdString(file);
        return true;
    }
    return false;
}
