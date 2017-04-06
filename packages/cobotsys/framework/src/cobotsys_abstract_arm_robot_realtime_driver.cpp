//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_arm_robot_realtime_driver.h"
#include "extra2.h"

namespace cobotsys {
AbstractArmRobotRealTimeDriver::AbstractArmRobotRealTimeDriver() {
}

AbstractArmRobotRealTimeDriver::~AbstractArmRobotRealTimeDriver() {
    INFO_DESTRUCTOR(this);
}

ArmRobotRealTimeStatusObserver::ArmRobotRealTimeStatusObserver() {
}

ArmRobotRealTimeStatusObserver::~ArmRobotRealTimeStatusObserver() {
    INFO_DESTRUCTOR(this);
}
}