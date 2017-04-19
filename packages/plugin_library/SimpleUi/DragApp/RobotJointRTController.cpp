//
// Created by 潘绪洋 on 17-3-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "RobotJointRTController.h"

RobotJointRTController::RobotJointRTController() : QObject(nullptr){
}

RobotJointRTController::~RobotJointRTController(){
    INFO_DESTRUCTOR(this);
}

void RobotJointRTController::onMoveFinish(uint32_t moveId){
}

void RobotJointRTController::onJointStatusUpdate(const std::vector<double>& jointPose){
    if (m_mutex.tryLock(8)) {
        m_curJoints = jointPose;
        m_mutex.unlock();
    }

    Q_EMIT jointUpdated();
}

void RobotJointRTController::onRobotConnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot){
}

void RobotJointRTController::onRobotDisconnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot){
}

void RobotJointRTController::getCurJoint(std::vector<double>& j){
    QMutexLocker locker(&m_mutex);
    j = m_curJoints;
}
