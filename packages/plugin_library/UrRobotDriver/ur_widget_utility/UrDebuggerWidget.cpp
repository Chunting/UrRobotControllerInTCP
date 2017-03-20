//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "UrDebuggerWidget.h"

UrDebuggerWidget::UrDebuggerWidget(){
    ui.setupUi(this);
    m_reverseMove = false;
    m_incBase = 1;

    connect(this, &UrDebuggerWidget::jointUpdated, this, &UrDebuggerWidget::updateJointValue);
}

UrDebuggerWidget::~UrDebuggerWidget(){
    INFO_DESTRUCTOR(this);
    stop();
}

bool UrDebuggerWidget::start(){
    if (m_robotDriver)
        m_robotDriver->start();
    return false;
}

void UrDebuggerWidget::pause(){
}

void UrDebuggerWidget::stop(){
    if (m_robotDriver)
        m_robotDriver->pause();
}

bool UrDebuggerWidget::setup(const QString& configFilePath){
    auto pObject = GlobalObjectFactory::instance()->createObject("UrRobotDriverFactory, Ver 1.0", "UrAdapter");
    m_robotDriver = std::dynamic_pointer_cast<cobotsys::AbstractRobotDriver>(pObject);

    QJsonObject json;
    loadJson(json, configFilePath);
    m_incBase = json["step_angle"].toDouble(1);

    if (m_robotDriver) {
        m_robotDriver->attach(std::dynamic_pointer_cast<RobotStatusObserver>(shared_from_this()));
        m_robotDriver->setup(configFilePath);
    }
    return true;
}

void UrDebuggerWidget::onMoveFinish(uint32_t moveId){
}

void UrDebuggerWidget::onJointStatusUpdate(const std::vector<double>& jointPose){
    m_mutex.lock();
    m_jointStatus = jointPose;
    m_mutex.unlock();
    Q_EMIT jointUpdated();

    if (m_robotDriver) {
        auto new_joint = jointPose;

        auto move_unit = m_incBase / 180 * CV_PI;
        if (m_reverseMove)
            move_unit *= -1;

        new_joint[1] += move_unit;
        if (new_joint[1] > CV_PI) {
            new_joint[1] = CV_PI;
            m_reverseMove = true;
        } else if (new_joint[1] < -CV_PI) {
            new_joint[1] = -CV_PI;
            m_reverseMove = false;
        }

        m_robotDriver->move(0, new_joint);
//        COBOT_LOG.info() << "Cur: " << jointPose[1] * 180 / CV_PI << ", Target: " << new_joint[1] * 180 / CV_PI;
    }
}

void UrDebuggerWidget::onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}

void UrDebuggerWidget::onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot){
}

void UrDebuggerWidget::closeEvent(QCloseEvent* event){
    stop();
    QWidget::closeEvent(event);
}

void UrDebuggerWidget::updateJointValue(){
    std::vector<double> jtmp;
    m_mutex.lock();
    jtmp = m_jointStatus;
    m_mutex.unlock();

    if (jtmp.size() > 0) ui.dsb_1->setValue(jtmp[0] * 180 / CV_PI);
    if (jtmp.size() > 1) ui.dsb_2->setValue(jtmp[1] * 180 / CV_PI);
    if (jtmp.size() > 2) ui.dsb_3->setValue(jtmp[2] * 180 / CV_PI);
    if (jtmp.size() > 3) ui.dsb_4->setValue(jtmp[3] * 180 / CV_PI);
    if (jtmp.size() > 4) ui.dsb_5->setValue(jtmp[4] * 180 / CV_PI);
    if (jtmp.size() > 5) ui.dsb_6->setValue(jtmp[5] * 180 / CV_PI);
}
