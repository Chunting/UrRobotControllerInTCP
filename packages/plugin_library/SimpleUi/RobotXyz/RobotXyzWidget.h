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
#include <QFileDialog>
#include <cobotsys_abstract_robot_driver.h>

using namespace cobotsys;

class RobotXyzWidget : public AbstractWidget {
Q_OBJECT
public:
    RobotXyzWidget();
    virtual ~RobotXyzWidget();

    void start();
    void stop();
    void loadRobotDriver();

public:
    virtual bool setup(const QString& configFilePath);
protected:
    void setupRobotDriver(ObjectGroup& objectGroup);
protected:
    Ui::RobotXyzWidget ui;
    AbstractRobotDriverPtr m_robotDriver;
};


#endif //PROJECT_ROBOTXYZWIDGET_H
