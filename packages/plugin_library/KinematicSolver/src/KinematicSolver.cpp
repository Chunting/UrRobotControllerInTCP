//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <extra2.h>
#include "KinematicSolver.h"

KinematicSolver::KinematicSolver(){

}

KinematicSolver::~KinematicSolver(){

}

bool KinematicSolver::solve(const vector<double>& cur, const vector<double>& target, vector<double>& result) {
	return true;
}
bool KinematicSolver::setup(const QString& configFilePath) {
	return true;
}