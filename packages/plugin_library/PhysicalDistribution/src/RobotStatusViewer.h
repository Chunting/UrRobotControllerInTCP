//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_ROBOTSTATUSVIEWER_H
#define COBOTSYS_ROBOTSTATUSVIEWER_H

#include <cobotsys_abstract_widget.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include "ui_RobotStatusViewer.h"
#include <mutex>
#include <QDoubleSpinBox>

using namespace cobotsys;

class RobotStatusViewer : public AbstractWidget, public ArmRobotRealTimeStatusObserver {
Q_OBJECT
public:
    RobotStatusViewer();
    virtual ~RobotStatusViewer();

    virtual bool setup(const QString& configFilePath);
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

Q_SIGNALS:
    void robotConnectionChanged(bool connected);
    void robotDataChanged();

protected:
    void handleUiConnection(bool bConnected);
    void handleRobotDataUpdate();

protected:
    Ui::RobotStatusViewer ui;
    std::mutex m_mutex;
    std::vector<double> m_joint;
    std::vector<QDoubleSpinBox*> m_boxs;
};


#endif //COBOTSYS_ROBOTSTATUSVIEWER_H
