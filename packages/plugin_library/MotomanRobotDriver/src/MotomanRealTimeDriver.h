//
// Created by 杨帆 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef MOTOMAN_REALTIME_DRIVER_H
#define MOTOMAN_REALTIME_DRIVER_H

#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <thread>

#include "CobotMotomanComm.h"
#include "CobotMotomanCommCtrl.h"
#include "CobotMotomanRealTimeCommCtrl.h"
#include "CobotMotomanDriver.h"
#include "CobotMotomanDigitIoAdapter.h"

using namespace cobotsys;

class MotomanRealTimeDriver : public QObject, public AbstractArmRobotRealTimeDriver {
Q_OBJECT
public:
    MotomanRealTimeDriver();
    virtual ~MotomanRealTimeDriver();

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
protected:
    std::mutex m_mutex;
    std::thread m_thread;
    bool m_isWatcherRunning;
    bool m_isStarted;

    std::condition_variable m_rt_msg_cond;
    std::condition_variable m_msg_cond;

    std::string m_attr_robot_ip;
    double m_attr_servoj_time;
    double m_attr_servoj_lookahead;
    double m_attr_servoj_gain;

    std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > m_observers;

    std::vector<double> m_curReqQ;
    bool m_curReqQValid;

    std::vector<double> m_robotJointQCache;

    CobotMotomanCommCtrl* m_ctrl;
    CobotMotomanRealTimeCommCtrl* m_rt_ctrl;

    CobotMotomanDriver* m_motomanDriver;

    std::shared_ptr<CobotMotomanDigitIoAdapter> m_digitInput;
    std::shared_ptr<CobotMotomanDigitIoAdapter> m_digitOutput;
};


#endif //MOTOMAN_REALTIME_DRIVER_H
