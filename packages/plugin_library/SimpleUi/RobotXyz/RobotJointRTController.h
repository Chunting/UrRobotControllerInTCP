//
// Created by 潘绪洋 on 17-3-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ROBOTJOINTRTCONTROLLER_H
#define PROJECT_ROBOTJOINTRTCONTROLLER_H

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <QtCore/QMutex>

using namespace cobotsys;

class RobotJointRTController : public QObject, public ArmRobotMoveStatusObserver {
Q_OBJECT
public:
    RobotJointRTController();
    virtual ~RobotJointRTController();

    virtual void onMoveFinish(uint32_t moveId);
    virtual void onJointStatusUpdate(const std::vector<double>& jointPose);
    virtual void onRobotConnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);
    virtual void onRobotDisconnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);

    void getCurJoint(std::vector<double>& j);


Q_SIGNALS:
    void jointUpdated();

protected:

protected:
    std::vector<double> m_curJoints;
    QMutex m_mutex;
};


#endif //PROJECT_ROBOTJOINTRTCONTROLLER_H
