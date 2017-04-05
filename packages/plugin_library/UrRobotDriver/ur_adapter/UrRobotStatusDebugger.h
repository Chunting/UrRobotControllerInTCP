//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URROBOTSTATUSDEBUGGER_H
#define PROJECT_URROBOTSTATUSDEBUGGER_H


#include <cobotsys_abstract_arm_robot_move_driver.h>

using namespace cobotsys;

class UrRobotStatusDebugger : public cobotsys::AbstractObject, public cobotsys::ArmRobotMoveStatusObserver {
public:
    UrRobotStatusDebugger();
    virtual ~UrRobotStatusDebugger();

    virtual void onMoveFinish(uint32_t moveId);
    virtual void onJointStatusUpdate(const std::vector<double>& jointPose);
    virtual void onRobotConnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);
    virtual void onRobotDisconnected(std::shared_ptr<AbstractArmRobotMoveDriver> pRobot);
    virtual bool setup(const QString& configFilePath);
protected:

    std::vector<double> m_oldJonitQ;
};


#endif //PROJECT_URROBOTSTATUSDEBUGGER_H
