//
// Created by longhuicai on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_forcecontrol_solver.h"


namespace cobotsys {
	AbstractForceControlSolver::AbstractForceControlSolver() : m_ptrKinematicSolver(nullptr){
}

	AbstractForceControlSolver::~AbstractForceControlSolver() {
}
}