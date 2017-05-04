//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef TUTORIALS_DEMOROBOTSTATUSLISTENER_H
#define TUTORIALS_DEMOROBOTSTATUSLISTENER_H

#include <cobotsys_abstract_arm_robot_realtime_driver.h>

using namespace cobotsys;

class Ur10JointFilter : public AbstractObject, public ArmRobotJointTargetFilter {
public:
    Ur10JointFilter();
    virtual ~Ur10JointFilter();
    virtual bool setup(const QString& configFilePath);
    virtual void applyFilter(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus);

protected:
    std::vector<double> m_target;
    std::vector<double> m_preTarget;
};


#endif //TUTORIALS_DEMOROBOTSTATUSLISTENER_H
