//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <cobotsys_abstract_object_factory.h>
#include "KinematicSolver.h"
#include <extra2.h>
#include <cobotsys_abstract_factory_macro.h>

COBOTSYS_FACTORY_BEGIN(KinematicSolverFactory)
        COBOTSYS_FACTORY_EXPORT(KinematicSolver)
COBOTSYS_FACTORY_END(KinematicSolverFactory, "1.0")
