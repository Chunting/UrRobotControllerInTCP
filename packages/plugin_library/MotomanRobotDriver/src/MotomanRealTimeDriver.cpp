//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonObject>
#include <extra2.h>
#include "MotomanRealTimeDriver.h"
#include "CobotMotoman.h"


MotomanRealTimeDriver::MotomanRealTimeDriver() : QObject(nullptr) {
    m_isWatcherRunning = false;
    m_isStarted = false;

    m_ctrl = nullptr;
    m_rt_ctrl = nullptr;
    m_motomanDriver = nullptr;
    m_curReqQ.clear();

    m_digitInput = make_shared<CobotMotomanDigitIoAdapter>();
    m_digitOutput = make_shared<CobotMotomanDigitIoAdapter>();
}

MotomanRealTimeDriver::~MotomanRealTimeDriver() {
    if (m_isWatcherRunning) {
        m_isWatcherRunning = false;
        m_rt_msg_cond.notify_all();
        m_thread.join();
    }
}

void MotomanRealTimeDriver::move(const std::vector<double>& q) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    if (m_isStarted) {
        m_curReqQ = q;
        m_curReqQValid = true;
    }
}

std::shared_ptr<AbstractDigitIoDriver> MotomanRealTimeDriver::getDigitIoDriver(int deviceId) {
    if (deviceId == 0)
        return m_digitOutput;
    if (deviceId == 1)
        return m_digitInput;
    return nullptr;
}

void MotomanRealTimeDriver::attach(const std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    for (auto& iter : m_observers) {
        if (iter.get() == observer.get()) {
            return; // Already have attached
        }
    }

    if (observer) {
        m_observers.push_back(observer);
    }
}

bool MotomanRealTimeDriver::start() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_motomanDriver) {
        COBOT_LOG.info() << "Already start, if want restart, stop first";
        return false;
    }
    m_motomanDriver = new CobotMotomanDriver(m_rt_msg_cond, m_msg_cond, m_attr_robot_ip.c_str());
    connect(m_motomanDriver, &CobotMotomanDriver::driverStartSuccess, this, &MotomanRealTimeDriver::handleDriverReady);
    connect(m_motomanDriver, &CobotMotomanDriver::driverStartFailed, this, &MotomanRealTimeDriver::handleDriverDisconnect);
    connect(m_motomanDriver, &CobotMotomanDriver::driverStopped, this, &MotomanRealTimeDriver::handleDriverDisconnect);
    connect(m_motomanDriver, &QObject::destroyed, [=](QObject*) { handleDriverDisconnect(); });
    m_motomanDriver->setServojTime(m_attr_servoj_time);
    m_motomanDriver->setServojLookahead(m_attr_servoj_lookahead);
    m_motomanDriver->setServojGain(m_attr_servoj_gain);
    m_motomanDriver->startDriver();

    // 这里是数字驱动的部分
    m_digitInput->setMotomanRealTimeCtrl(m_motomanDriver->m_motomanRealTimeCommCtrl);
    m_digitOutput->setMotomanRealTimeCtrl(m_motomanDriver->m_motomanRealTimeCommCtrl);
    m_digitInput->m_isInput = true;
    m_digitOutput->m_isOutput = true;
    return true;
}

void MotomanRealTimeDriver::stop() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_motomanDriver) {
        m_curReqQValid = false;
        m_curReqQ.clear();
        m_isStarted = false;
        m_motomanDriver->stopDriver();
        m_motomanDriver->deleteLater();
        m_motomanDriver = nullptr;
        m_digitInput->setMotomanRealTimeCtrl(nullptr);
        m_digitOutput->setMotomanRealTimeCtrl(nullptr);
    }
}

bool MotomanRealTimeDriver::setup(const QString& configFilePath) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    auto success = _setup(configFilePath);

    if (success) {
    } else {
        m_observers.clear(); // detach all observer
    }
    return success;
}

void MotomanRealTimeDriver::robotStatusWatcher() {
    std::mutex m;
    std::unique_lock<std::mutex> lck(m);

    auto time_cur = std::chrono::high_resolution_clock::now();

    auto pStatus = std::make_shared<ArmRobotStatus>();
    std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > observer_tmp;
    std::vector<double> q_next;

    COBOT_LOG.notice() << "Motoman Status Watcher is Running.";
    while (m_isWatcherRunning) {
        m_rt_msg_cond.wait(lck);

        if (m_mutex.try_lock()) {
            _updateDigitIoStatus();
            m_mutex.unlock();
        }


        // 计算时间间隔
        auto time_rdy = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_diff = time_rdy - time_cur; // 时间间隙
        time_cur = time_rdy;
        //COBOT_LOG.info() << "Status Updated: " << time_diff.count();

        // 抓取当前姿态
        if (m_mutex.try_lock()) {
            if (m_motomanDriver) {
                auto pState = m_motomanDriver->m_motomanRealTimeCommCtrl->motoman->getRobotState();
                q_next = pState->getQActual();
                m_robotJointQCache = q_next;
            }
            m_mutex.unlock();
        }
        pStatus->q_actual = q_next;


        // 通知所有观察者，机器人数据已经更新。
        if (m_isStarted) {
            notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
                observer->onArmRobotStatusUpdate(pStatus);
            });
        }

        // 获取当前控制数据
        if (m_mutex.try_lock()) {
            if (m_curReqQValid) {
                q_next = m_curReqQ;
            }
            m_mutex.unlock();
        }

        // 更新控制驱动数据, 如果没有数据，默认会是当前状态。
        if (m_isStarted && q_next.size() >= CobotMotoman::JOINT_NUM_) {
            if (m_mutex.try_lock()) {
                m_motomanDriver->servoj(q_next);
                m_mutex.unlock();
            }
            //auto info_log = COBOT_LOG.info();
            //for (int i = 0; i < (int)q_next.size(); i++) {
            //	info_log << q_next[i] / M_PI * 180 << ", ";
            //}
        }
    }
    COBOT_LOG.notice() << "Motoman Status Watcher shutdown!";
}

bool MotomanRealTimeDriver::_setup(const QString& configFilePath) {
    QJsonObject json;
    //TODO Maybe it need to be modified.(motoman)
    if (loadJson(json, configFilePath)) {
        m_attr_robot_ip = json["robot_ip"].toString("localhost").toStdString();
        m_attr_servoj_time = json["servoj_time"].toDouble(0.08);
        m_attr_servoj_lookahead = json["servoj_lookahead"].toDouble(0.05);
        m_attr_servoj_gain = json["servoj_gain"].toDouble(300);

        m_isWatcherRunning = true;
        m_thread = std::thread(&MotomanRealTimeDriver::robotStatusWatcher, this);
        return true;
    }
    return false;
}

void MotomanRealTimeDriver::handleDriverReady() {
    m_isStarted = true;
    notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
        observer->onArmRobotConnect();
    });
}

QString MotomanRealTimeDriver::getRobotUrl() {
    return m_attr_robot_ip.c_str();
}

void MotomanRealTimeDriver::handleDriverDisconnect() {
    COBOT_LOG.info() << "MotomanRealTimeDriver Disconnect";
    stop();
    notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
        observer->onArmRobotDisconnect();
    });
}

void MotomanRealTimeDriver::notify(std::function<void(std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer)> func) {
    if (func) {
        std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > observer_tmp;
        if (m_mutex.try_lock()) {
            observer_tmp = m_observers;
            m_mutex.unlock();
        }

        for (auto& observer : observer_tmp) {
            func(observer);
        }
    }
}

void MotomanRealTimeDriver::_updateDigitIoStatus() {
    if (m_digitInput && m_isStarted && m_motomanDriver) {
        auto outBits = m_motomanDriver->m_motomanCommCtrl->motoman->getRobotState()->getDigitalOutputBits();
        auto inBits = m_motomanDriver->m_motomanCommCtrl->motoman->getRobotState()->getDigitalInputBits();

        m_digitInput->m_inputIoStatus = inBits;
        m_digitOutput->m_outputIoStatus = outBits;

        m_digitInput->debugIoStatus();
        m_digitOutput->debugIoStatus();
    }
}

void MotomanRealTimeDriver::clearAttachedObject() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_observers.clear();
}

std::vector<double> MotomanRealTimeDriver::getRobotJointQ() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    return m_robotJointQCache;
}
