//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URREALTIMEDRIVER_H
#define PROJECT_URREALTIMEDRIVER_H

#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <thread>
#include "CobotUrComm.h"
#include "CobotUrCommCtrl.h"
#include "CobotUrRealTimeCommCtrl.h"
#include "CobotUrDriver.h"
#include "CobotUrDigitIoAdapter.h"

using namespace cobotsys;

class URRealTimeDriver : public QObject, public AbstractArmRobotRealTimeDriver {
Q_OBJECT
public:
    URRealTimeDriver();
    virtual ~URRealTimeDriver();

    virtual void move(const std::vector<double>& q);
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);
    virtual void attach(const std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer);
    virtual bool start();
    virtual void stop();
    virtual bool setup(const QString& configFilePath);
    virtual QString getRobotUrl();
    virtual void clearAttachedObject();
    virtual std::vector<double> getRobotJointQ();
protected:
    void robotStatusWatcher();

    bool _setup(const QString& configFilePath);

    void handleDriverReady();
    void handleDriverDisconnect();
    void notify(std::function<void(std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer)> func);

    void _updateDigitIoStatus();

    void handleObjectDestroy(QObject* object);
protected:
    std::mutex m_mutex;
    std::thread m_thread;
    bool m_isWatcherRunning;
    bool m_isStarted;

    std::string m_attr_robot_ip;
    double m_attr_servoj_time;
    double m_attr_servoj_lookahead;
    double m_attr_servoj_gain;

    std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > m_observers;

    std::vector<double> m_curReqQ;
    bool m_curReqQValid;

    std::vector<double> m_robotJointQCache;

    CobotUrDriver* m_urDriver;

    std::shared_ptr<std::condition_variable> m_urMessage;

    std::shared_ptr<CobotUrDigitIoAdapter> m_digitInput;
    std::shared_ptr<CobotUrDigitIoAdapter> m_digitOutput;

    std::shared_ptr<bool> m_objectAlive;
};


#endif //PROJECT_URREALTIMEDRIVER_H
