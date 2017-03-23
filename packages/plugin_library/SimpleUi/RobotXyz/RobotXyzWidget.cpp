//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "RobotXyzWidget.h"


RobotXyzWidget::RobotXyzWidget(){
    ui.setupUi(this);

    connect(ui.btnLoadDriver, &QPushButton::released, this, &RobotXyzWidget::loadRobotDriver);
}

RobotXyzWidget::~RobotXyzWidget(){
}

void RobotXyzWidget::start(){
}

void RobotXyzWidget::stop(){
}

void RobotXyzWidget::loadRobotDriver(){
    QString filename = QFileDialog::getOpenFileName(this);
    if (filename.size()) {
        QJsonObject json;
        if (loadJson(json, filename)){
            m_robotDriver.reset();
            ObjectGroup objectGroup;
            if (objectGroup.init(json)) {
                setupRobotDriver(objectGroup);
            }
        }
    }
}

void RobotXyzWidget::setupRobotDriver(ObjectGroup& objectGroup){
    auto robotDriver = std::dynamic_pointer_cast<AbstractRobotDriver>(objectGroup.getObject("Robot"));
    if (robotDriver) {
        m_robotDriver = robotDriver;
    }
}

bool RobotXyzWidget::setup(const QString& configFilePath){
    return true;
}
