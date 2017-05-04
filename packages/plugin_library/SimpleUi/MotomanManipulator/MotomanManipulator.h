//
// Created by 杨帆 on 17-5-4.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_MOTOMAN_MANIPULATOR_H
#define PROJECT_MOTOMAN_MANIPULATOR_H

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
#include "ui_MotomanManipulator.h"

using namespace cobotsys;

class MotomanManipulator : public AbstractWidget, public ArmRobotRealTimeStatusObserver {
Q_OBJECT
public:
    MotomanManipulator();
    virtual ~MotomanManipulator();

    virtual bool setup(const QString& configFilePath);


    void handleSliderChange();
    void handleTargetChange();

    void createRobot();

    void startRobot();
    void stopRobot();

    void revertIoPorts();

Q_SIGNALS:
    void updateActualQ();
    void robotConnectStateChanged(bool isConnected);
protected:
    virtual void closeEvent(QCloseEvent* event);

public:
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
    void handleRobotState(bool isConnected);

    void setupCreationList();
    void onActualQUpdate();
    void updateTargetQ();

    void onRecTarget();
    void onGoTarget();
    void onPosiB();
    void onGoPosiB();
    void onLoopAB();

    void loopAbProg();
    void updateTargetQToUi();

    std::vector<double> getUiTarget();
    void goTarget(const std::vector<double>& targetq);
protected:
    int m_joint_num;
    bool m_noHandleChange;
    int m_initUIData;

    Ui::MotomanManipulator ui;

    std::vector<QSlider*> m_sliders;
    std::vector<QDoubleSpinBox*> m_target;
    std::vector<QDoubleSpinBox*> m_actual;

    std::vector<double> m_targetToGo;

    QStringList m_defaultRobotInfo;

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;

    std::vector<double> m_actualValue;
    std::mutex m_mutex;

    std::vector<double> m_PosiA;
    std::vector<double> m_PosiB;
    std::vector<double> m_loopQ;
    std::vector<double> m_qActual;

    std::vector<std::vector<double> > m_loopQQueue;
    size_t m_loopQQueueIndex;
    bool m_loopAB;

    QString m_groupBoxDefTitle;
};


#endif //PROJECT_MOTOMAN_MANIPULATOR_H
