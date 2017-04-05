//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URDEBUGGERWIDGET_H
#define PROJECT_URDEBUGGERWIDGET_H

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <cobotsys_abstract_controller.h>
#include <QCloseEvent>
#include "ui_UrDebuggerWidget.h"
#include <chrono>


using namespace cobotsys;
class UrDebuggerWidget : public cobotsys::AbstractControllerWidget,
                         public ArmRobotMoveStatusObserver {
Q_OBJECT
public:
    UrDebuggerWidget();
    virtual ~UrDebuggerWidget();

    virtual bool start();
    virtual void pause();
    virtual void stop();

    virtual bool setup(const QString& configFilePath);

Q_SIGNALS:
    void jointUpdated();


public:
    void updateJointValue();
protected:
    virtual void closeEvent(QCloseEvent* event);

public:
    virtual void onMoveFinish(uint32_t moveId);
    virtual void onJointStatusUpdate(const std::vector<double>& jointPose);
    virtual void onRobotConnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);
    virtual void onRobotDisconnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);


protected:
    void setMoveTarget(int);
    void debugMove(int jointId);
    void curAsTarget();

    bool moveTarget();
protected:
    std::shared_ptr<AbstractArmRobotMoveDriver> m_robotDriver;
    bool m_reverseMove;
    double m_incBase;
    Ui::RobotDebugger ui;

    std::vector<double> m_jointStatus;
    std::mutex m_mutex;

    double m_singleMoveRangeLow;
    double m_singleMoveRangeHigh;
    bool m_doAction;

    std::chrono::high_resolution_clock::time_point m_timeLastJU;

    std::deque<double> m_timeArray;

    std::vector<double> m_jointTarget;
    bool m_doSliderMove;
};


#endif //PROJECT_URDEBUGGERWIDGET_H
