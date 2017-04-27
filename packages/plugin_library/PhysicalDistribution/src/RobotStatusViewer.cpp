//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "RobotStatusViewer.h"


RobotStatusViewer::RobotStatusViewer() {
    ui.setupUi(this);
    connect(this, &RobotStatusViewer::robotConnectionChanged, this, &RobotStatusViewer::handleUiConnection);
    connect(this, &RobotStatusViewer::robotDataChanged, this, &RobotStatusViewer::handleRobotDataUpdate);
    handleUiConnection(false);
    m_boxs.push_back(ui.doubleSpinBox_1);
    m_boxs.push_back(ui.doubleSpinBox_2);
    m_boxs.push_back(ui.doubleSpinBox_3);
    m_boxs.push_back(ui.doubleSpinBox_4);
    m_boxs.push_back(ui.doubleSpinBox_5);
    m_boxs.push_back(ui.doubleSpinBox_6);
}

RobotStatusViewer::~RobotStatusViewer() {
    INFO_DESTRUCTOR(this);
}

bool RobotStatusViewer::setup(const QString& configFilePath) {
    return true;
}

void RobotStatusViewer::onArmRobotConnect() {
    Q_EMIT robotConnectionChanged(true);
}

void RobotStatusViewer::onArmRobotDisconnect() {
    Q_EMIT robotConnectionChanged(false);
}

void RobotStatusViewer::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    m_mutex.lock();
    m_joint = ptrRobotStatus->q_actual;
    m_mutex.unlock();
    Q_EMIT robotDataChanged();
}

void RobotStatusViewer::handleUiConnection(bool bConnected) {
    if (bConnected) {
        ui.labelStatus->setText(tr("Robot Connected"));
    } else {
        ui.labelStatus->setText(tr("Robot Disconnected"));
    }
}

void RobotStatusViewer::handleRobotDataUpdate() {
    size_t loop = smaller(m_joint.size(), m_boxs.size());

    for (size_t i = 0; i < loop; i++) {
        m_boxs[i]->setValue(m_joint[i] * 180 / M_PI);
    }
}
