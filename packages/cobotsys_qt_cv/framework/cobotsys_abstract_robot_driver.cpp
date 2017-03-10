//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_robot_driver.h"

namespace cobotsys {
AbstractRobotDriver::AbstractRobotDriver(){
}

AbstractRobotDriver::~AbstractRobotDriver(){
}

uint32_t AbstractRobotDriver::generateMoveId(){
    static uint32_t moveId = 0;
    moveId++;
    return moveId;
}
}