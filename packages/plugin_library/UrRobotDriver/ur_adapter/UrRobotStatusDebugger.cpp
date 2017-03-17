//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "UrRobotStatusDebugger.h"

UrRobotStatusDebugger::UrRobotStatusDebugger(){
}

UrRobotStatusDebugger::~UrRobotStatusDebugger(){
}

void UrRobotStatusDebugger::onMoveFinish(uint32_t moveId){
}

void UrRobotStatusDebugger::onJointStatusUpdate(const std::vector<double>& jointPose){
    bool is_new_joint_q = m_oldJonitQ.size() != jointPose.size();
    double q_diff = 0;
    if (m_oldJonitQ.size() == jointPose.size()) {
        for (size_t i = 0; i < m_oldJonitQ.size(); i++) {
            q_diff = fabs(m_oldJonitQ[i] - jointPose[i]);
            if (q_diff > 0.0001) {

                is_new_joint_q = true;
                break;
            }
        }
    }

    m_oldJonitQ = jointPose;
    if (m_oldJonitQ.size() >= 6 && is_new_joint_q) {
        COBOT_LOG.info() << "Joint Update: "
                         << m_oldJonitQ[0] / CV_PI * 180 << ", "
                         << m_oldJonitQ[1] / CV_PI * 180 << ", "
                         << m_oldJonitQ[2] / CV_PI * 180 << ", "
                         << m_oldJonitQ[3] / CV_PI * 180 << ", "
                         << m_oldJonitQ[4] / CV_PI * 180 << ", "
                         << m_oldJonitQ[5] / CV_PI * 180;
    }
}

void UrRobotStatusDebugger::onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}

void UrRobotStatusDebugger::onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}
