//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <mutex>
#include "cobotsys_abstract_arm_robot_move_driver.h"


namespace cobotsys {
ArmRobotMoveStatusObserver::ArmRobotMoveStatusObserver(){
}

ArmRobotMoveStatusObserver::~ArmRobotMoveStatusObserver(){
}
}

namespace cobotsys {
AbstractArmRobotMoveDriver::AbstractArmRobotMoveDriver(){
}

AbstractArmRobotMoveDriver::~AbstractArmRobotMoveDriver(){
}

uint32_t AbstractArmRobotMoveDriver::generateMoveId(){
    static uint32_t moveId = 0;
    static std::mutex moveIdMutex;
    std::lock_guard<std::mutex> lock_guard(moveIdMutex);
    moveId++;
    return moveId;
}
}