//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URREALTIMEDRIVER_H
#define PROJECT_URREALTIMEDRIVER_H

#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>

using namespace cobotsys;

class URRealTimeDriver : public AbstractArmRobotRealTimeDriver {
public:
    URRealTimeDriver();
    virtual ~URRealTimeDriver();


    virtual void move(const std::vector<double>& q);
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);
    virtual void attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer);
    virtual bool start();
    virtual void stop();
    virtual bool setup(const QString& configFilePath);

protected:
    std::mutex m_mutex;
};


#endif //PROJECT_URREALTIMEDRIVER_H
