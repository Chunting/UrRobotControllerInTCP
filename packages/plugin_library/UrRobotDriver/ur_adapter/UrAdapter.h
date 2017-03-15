//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URADAPTER_H
#define PROJECT_URADAPTER_H


#include <cobotsys_abstract_robot_driver.h>
#include <cobotsys_observer_template.h>
#include "ur_driver.h"

using namespace cobotsys;

class UrAdapter : public cobotsys::AbstractRobotDriver {
public:
    UrAdapter();
    virtual ~UrAdapter();

public:
    virtual void move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal);

    virtual void attach(std::shared_ptr<RobotStatusObserver> observer);
    virtual void directControl(const std::string& command);

    virtual std::shared_ptr<QIODevice> getSerialIoDevice(int devicdId = 0);
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);

    virtual bool getMoveData(uint32_t moveId, cv::Point3d& pos, cv::Vec3d& normal);
    virtual void clearMoveDataHistory();

    virtual bool setup(const QString& configFilePath);
    virtual void start();
    virtual void pause();

protected:
    std::vector<std::shared_ptr<RobotStatusObserver> > m_observerArray;
    std::shared_ptr<UrDriver> m_urDriver;

    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;
};


#endif //PROJECT_URADAPTER_H
