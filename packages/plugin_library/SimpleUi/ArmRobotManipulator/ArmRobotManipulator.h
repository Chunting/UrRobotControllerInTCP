//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ARMROBOTMANIPULATOR_H
#define PROJECT_ARMROBOTMANIPULATOR_H

#include <QObject>
#include <cobotsys_abstract_widget.h>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QSpinBox>
#include <QJsonObject>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include "ui_ArmRobotManipulator.h"

using namespace cobotsys;

class ArmRobotManipulator : public AbstractWidget {
    Q_OBJECT
public:
    ArmRobotManipulator();
    virtual ~ArmRobotManipulator();

    virtual bool setup(const QString& configFilePath);


    void handleSliderChange();
    void handleTargetChange();

    void createRobot();

protected:
    int m_joint_num;
    bool m_noHandleChange;

    Ui::ArmRobotManipulator ui;

    std::vector<QSlider*> m_sliders;
    std::vector<QDoubleSpinBox*> m_target;
    std::vector<QDoubleSpinBox*> m_actual;

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
};


#endif //PROJECT_ARMROBOTMANIPULATOR_H
