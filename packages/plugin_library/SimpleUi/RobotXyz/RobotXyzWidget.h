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
#include <QPushButton>
#include <QDoubleSpinBox>
#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_kinematic_solver.h>
#include <cobotsys_file_finder.h>
#include <vector>

using namespace cobotsys;

class RobotXyzWidget : public AbstractWidget, public ArmRobotRealTimeStatusObserver {
    Q_OBJECT
public:
    RobotXyzWidget();
    virtual ~RobotXyzWidget();

    void start();
    void stop();


    void initUiComboList();

    void createArmRobotDriver();
    void createSolver();

	void copyCurXyzRpy();

Q_SIGNALS:
    void jointValueUpdated();

protected:
    virtual void closeEvent(QCloseEvent* event);
public:
    virtual bool setup(const QString& configFilePath);


public:
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
    void onJointUpdate();
    void onGoCommand();

protected:
    Ui::RobotXyzWidget ui;
    std::vector<QDoubleSpinBox*> m_dsbJointVals;

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
    std::shared_ptr<AbstractKinematicSolver> m_ptrSolver;

    std::mutex m_mutex;
    std::vector<double> m_jointValues;
};


#endif //PROJECT_ROBOTXYZWIDGET_H
