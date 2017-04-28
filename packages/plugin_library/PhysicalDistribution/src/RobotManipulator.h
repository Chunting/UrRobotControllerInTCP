//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_ROBOTMANIPULATOR_H
#define COBOTSYS_ROBOTMANIPULATOR_H

#include <cobotsys_abstract_widget.h>
#include "ui_RobotManipulator.h"
#include <QDoubleSpinBox>
#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <QTreeWidgetItem>
#include <QTreeWidget>

using namespace cobotsys;

class RobotManipulator :
        public AbstractWidget,
        public AbstractManipulator {
Q_OBJECT
public:
    RobotManipulator();
    virtual ~RobotManipulator();

    virtual bool setup(const QString& configFilePath);
    virtual void clearAttachedObject();


    virtual void setRobotMover(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& robot,
                               const std::shared_ptr<AbstractArmRobotMoveDriver>& mover);


    void onButtonResetJoint();
    void onButtonGo();
    void onButtonSetHome();
    void onButtonSetWaypoint();
    void onButtonGoHome();


    bool isBaseUnitType() const;

    std::vector<double> convertJoint(const std::vector<double>& joint);
    std::vector<double> getUiTargetValue() const;
protected:
    Ui::RobotManipulator ui;
    std::vector<QDoubleSpinBox*> m_spinValues;

    std::shared_ptr<AbstractArmRobotMoveDriver> m_ptrMover;
    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
};


#endif //COBOTSYS_ROBOTMANIPULATOR_H
