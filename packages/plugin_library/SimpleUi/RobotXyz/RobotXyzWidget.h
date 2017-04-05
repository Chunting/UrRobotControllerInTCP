//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ROBOTXYZWIDGET_H
#define PROJECT_ROBOTXYZWIDGET_H

#include <cobotsys_abstract_widget.h>
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include "ui_RobotXyzWidget.h"
#include "RobotJointRTController.h"
#include <QFileDialog>
#include <cobotsys_abstract_arm_robot_move_driver.h>

using namespace cobotsys;

class RobotXyzWidget : public AbstractWidget {
Q_OBJECT
public:
    RobotXyzWidget();
    virtual ~RobotXyzWidget();

    void start();
    void stop();
    void loadRobotDriver();

protected:
    virtual void closeEvent(QCloseEvent* event);
public:
    virtual bool setup(const QString& configFilePath);
protected:
    void setupRobotDriver(ObjectGroup& objectGroup);

protected:
    void updateUiJointValue();

protected:
    Ui::RobotXyzWidget ui;
    AbstractArmRobotMoveDriverPtr m_robotDriver;
    std::shared_ptr<RobotJointRTController> m_jrtController;
    std::vector<QDoubleSpinBox*> m_dsbJointVals;
};


#endif //PROJECT_ROBOTXYZWIDGET_H
