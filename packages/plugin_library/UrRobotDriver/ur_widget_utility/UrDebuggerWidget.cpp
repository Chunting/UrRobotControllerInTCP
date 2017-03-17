//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "UrDebuggerWidget.h"

UrDebuggerWidget::UrDebuggerWidget(){
}

UrDebuggerWidget::~UrDebuggerWidget(){
}

bool UrDebuggerWidget::start(){
    if (m_robotDriver)
        m_robotDriver->start();
    return false;
}

void UrDebuggerWidget::pause(){
}

void UrDebuggerWidget::stop(){
}

bool UrDebuggerWidget::setup(const QString& configFilePath){
    auto pObject = GlobalObjectFactory::instance()->createObject("UrRobotDriverFactory, Ver 1.0", "UrAdapter");
    m_robotDriver = std::dynamic_pointer_cast<cobotsys::AbstractRobotDriver>(pObject);
    if (m_robotDriver) {
        m_robotDriver->setup(configFilePath);
        m_robotDriver->attach(std::dynamic_pointer_cast<RobotStatusObserver>(shared_from_this()));
    }
    return true;
}

void UrDebuggerWidget::onMoveFinish(uint32_t moveId){
}

void UrDebuggerWidget::onJointStatusUpdate(const std::vector<double>& jointPose){
    if (m_robotDriver) {
        m_robotDriver->move(0, {0, 0, 0, 0, 0, 0});
    }
}

void UrDebuggerWidget::onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}

void UrDebuggerWidget::onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}
