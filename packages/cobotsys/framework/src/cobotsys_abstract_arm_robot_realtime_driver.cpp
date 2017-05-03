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

bool AbstractArmRobotRealTimeDriver::setTargetJointFilter(const std::shared_ptr<ArmRobotJointTargetFilter>& filter) {
    COBOT_LOG.debug() << "setTargetJointFilter is not Implement";
    return false;
}

ArmRobotRealTimeStatusObserver::ArmRobotRealTimeStatusObserver() {
}

ArmRobotRealTimeStatusObserver::~ArmRobotRealTimeStatusObserver() {
    INFO_DESTRUCTOR(this);
}

ArmRobotJointTargetFilter::ArmRobotJointTargetFilter() {
}

ArmRobotJointTargetFilter::~ArmRobotJointTargetFilter() {
    INFO_DESTRUCTOR(this);
}
}