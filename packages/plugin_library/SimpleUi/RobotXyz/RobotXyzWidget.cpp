//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "RobotXyzWidget.h"


RobotXyzWidget::RobotXyzWidget(){
    ui.setupUi(this);

    connect(ui.btnLoadDriver, &QPushButton::released, this, &RobotXyzWidget::loadRobotDriver);
    m_jrtController = std::make_shared<RobotJointRTController>();

    connect(m_jrtController.get(), &RobotJointRTController::jointUpdated, this, &RobotXyzWidget::updateUiJointValue);
    connect(ui.btnStart, &QPushButton::released, this, &RobotXyzWidget::start);
    connect(ui.btnStop, &QPushButton::released, this, &RobotXyzWidget::stop);

    m_dsbJointVals.push_back(ui.dsb_j0);
    m_dsbJointVals.push_back(ui.dsb_j1);
    m_dsbJointVals.push_back(ui.dsb_j2);
    m_dsbJointVals.push_back(ui.dsb_j3);
    m_dsbJointVals.push_back(ui.dsb_j4);
    m_dsbJointVals.push_back(ui.dsb_j5);
    for (auto& dsb : m_dsbJointVals) {
    }
}

RobotXyzWidget::~RobotXyzWidget(){
    INFO_DESTRUCTOR(this);
}

void RobotXyzWidget::start(){
}

void RobotXyzWidget::stop(){
}

void RobotXyzWidget::loadRobotDriver(){
    QString filename = QFileDialog::getOpenFileName(this);
    if (filename.size()) {
        QJsonObject json;
        if (loadJson(json, filename)) {
            m_robotDriver.reset();
            ObjectGroup objectGroup;
            if (objectGroup.init(json)) {
                setupRobotDriver(objectGroup);
            }
        }
    }
}

void RobotXyzWidget::setupRobotDriver(ObjectGroup& objectGroup){
    auto robotDriver = std::dynamic_pointer_cast<AbstractArmRobotMoveDriver>(objectGroup.getObject("Robot"));
    if (robotDriver) {
        m_robotDriver = robotDriver;
        m_robotDriver->attach(m_jrtController);
    }
}

bool RobotXyzWidget::setup(const QString& configFilePath){
    return true;
}

void RobotXyzWidget::updateUiJointValue(){
    std::vector<double> jointVals;
    m_jrtController->getCurJoint(jointVals);

    if (jointVals.size() <= m_dsbJointVals.size()) {
        for (int i = 0; i < (int) jointVals.size(); i++) {
            m_dsbJointVals[i]->setValue(jointVals[i]);
        }
    }
}

void RobotXyzWidget::closeEvent(QCloseEvent* event){
    m_jrtController.reset();
    m_robotDriver.reset();
    QWidget::closeEvent(event);
}
