//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EPSONROBOTDRIVER_H
#define PROJECT_EPSONROBOTDRIVER_H

#include <cobotsys_abstract_arm_robot_realtime_driver.h>

using namespace cobotsys;

class EpsonRobotDriver : public AbstractArmRobotRealTimeDriver {
public:
    EpsonRobotDriver();
    virtual ~EpsonRobotDriver();

public:
    virtual void move(const std::vector<double>& q);
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);
    virtual void attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer);
    virtual bool start();
    virtual void stop();
    virtual bool setup(const QString& configFilePath);
};


#endif //PROJECT_EPSONROBOTDRIVER_H
