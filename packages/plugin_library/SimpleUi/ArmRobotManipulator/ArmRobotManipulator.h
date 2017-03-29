//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ARMROBOTMANIPULATOR_H
#define PROJECT_ARMROBOTMANIPULATOR_H

#include <mutex>
#include <QObject>
#include <cobotsys_abstract_widget.h>
#include <QCloseEvent>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QSpinBox>
#include <QJsonObject>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include "ui_ArmRobotManipulator.h"

using namespace cobotsys;

class ArmRobotManipulator : public AbstractWidget, public ArmRobotRealTimeStatusObserver {
Q_OBJECT
public:
    ArmRobotManipulator();
    virtual ~ArmRobotManipulator();

    virtual bool setup(const QString& configFilePath);


    void handleSliderChange();
    void handleTargetChange();

    void createRobot();

    void startRobot();
    void stopRobot();


Q_SIGNALS:
    void updateActualQ();
protected:
    virtual void closeEvent(QCloseEvent* event);

public:
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
    void setupCreationList();
    void onActualQUpdate();
    void updateTargetQ();

    void onRecTarget();
    void onGoTarget();
protected:
    int m_joint_num;
    bool m_noHandleChange;
    int m_initUIData;

    Ui::ArmRobotManipulator ui;

    std::vector<QSlider*> m_sliders;
    std::vector<QDoubleSpinBox*> m_target;
    std::vector<QDoubleSpinBox*> m_actual;

    QStringList m_defaultRobotInfo;

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;

    std::vector<double> m_actualValue;
    std::mutex m_mutex;

    std::vector<double> m_recTarget;

    QString m_groupBoxDefTitle;
};


#endif //PROJECT_ARMROBOTMANIPULATOR_H
