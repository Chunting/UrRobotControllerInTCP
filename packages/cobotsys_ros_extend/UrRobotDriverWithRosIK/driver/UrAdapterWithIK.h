//
// Created by 潘绪洋 on 17-3-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URADAPTERWITHIK_H
#define PROJECT_URADAPTERWITHIK_H


#include <cobotsys_abstract_arm_robot_move_driver.h>
#include "../../../plugin_library/UrRobotDriver/src/ur_driver.h"
#include <UrStatusWatcher2.h>
#include <QTimer>
#include <ros_moveit_wrapper.h>

using namespace cobotsys;

class UrAdapterWithIK : public QObject, public cobotsys::AbstractRobotDriver {
Q_OBJECT
public:
    UrAdapterWithIK();
    virtual ~UrAdapterWithIK();

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

    virtual bool isReady() const;

    std::shared_ptr<UrDriver>& getDriver(){ return m_urDriver; }

    void notify(std::function<void(std::shared_ptr<RobotStatusObserver>&)> applyFunc);

    void tickCheckService();

    bool connectedOnceSetup();
protected:
    std::vector<std::shared_ptr<RobotStatusObserver> > m_observerArray;
    std::shared_ptr<UrDriver> m_urDriver;

    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;

    std::shared_ptr<UrStatusWatcher2> m_urWatcher;

    std::mutex m_resMutex;

    bool m_isStarted;

    QTimer* m_pConnectionCheckTimer;
    bool m_connectionNotifyStatus;
    bool m_disconnectNotifyStatus;

    bool m_onceStartCall;
};

#endif //PROJECT_URADAPTERWITHIK_H
