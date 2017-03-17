//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URADAPTER_H
#define PROJECT_URADAPTER_H


#include <cobotsys_abstract_robot_driver.h>
#include <cobotsys_observer_template.h>
#include "ur_driver.h"
#include <UrStatusWatcher.h>

using namespace cobotsys;

class UrAdapter : public cobotsys::AbstractRobotDriver {
public:
    UrAdapter();
    virtual ~UrAdapter();

public:
    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal);
    virtual void move(uint32_t moveId, const std::vector<double>& jointPos);

    virtual void attach(std::shared_ptr<RobotStatusObserver> observer);
    virtual void directControl(const std::string& command);

    virtual std::shared_ptr<QIODevice> getSerialIoDevice(int devicdId = 0);
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);

    virtual bool getMoveData(uint32_t moveId, std::vector<double>& moveData, int& moveDataType);
    virtual void clearMoveDataHistory();

    virtual bool setup(const QString& configFilePath);
    virtual bool start();
    virtual void pause();


    std::shared_ptr<UrDriver>& getDriver(){ return m_urDriver; }

    void notify(std::function<void(std::shared_ptr<RobotStatusObserver>&)> applyFunc);
protected:
    std::vector<std::shared_ptr<RobotStatusObserver> > m_observerArray;
    std::shared_ptr<UrDriver> m_urDriver;

    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;

    std::shared_ptr<UrStatusWatcher> m_urWatcher;

    std::mutex m_resMutex;

    bool m_isStarted;
};


#endif //PROJECT_URADAPTER_H
