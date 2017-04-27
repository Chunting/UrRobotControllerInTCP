//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "RobotManipulator.h"

RobotManipulator::RobotManipulator() {
    ui.setupUi(this);
}

RobotManipulator::~RobotManipulator() {
    INFO_DESTRUCTOR(this);
}

bool RobotManipulator::setup(const QString& configFilePath) {
    return true;
}

void RobotManipulator::clearAttachedObject() {
}
