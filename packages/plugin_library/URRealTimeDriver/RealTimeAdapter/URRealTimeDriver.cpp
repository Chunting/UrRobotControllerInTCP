//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonObject>
#include <extra2.h>
#include "URRealTimeDriver.h"
#include "CobotUr.h"


URRealTimeDriver::URRealTimeDriver() : QObject(nullptr) {
    m_isWatcherRunning = false;
    m_isStarted = false;

    m_urDriver = nullptr;
    m_curReqQ.clear();

    m_digitInput = std::make_shared<CobotUrDigitIoAdapter>();
    m_digitOutput = std::make_shared<CobotUrDigitIoAdapter>();
    m_objectAlive = std::make_shared<bool>(true);
    m_urMessage = std::make_shared<std::condition_variable>();
}

URRealTimeDriver::~URRealTimeDriver() {
    if (m_isWatcherRunning) {
        m_isWatcherRunning = false;
        m_urMessage->notify_all();
        m_thread.join();
    }
    *m_objectAlive = false;
}

void URRealTimeDriver::move(const std::vector<double>& q) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    if (m_isStarted) {
        m_curReqQ = q;
        m_curReqQValid = true;
    }
}

std::shared_ptr<AbstractDigitIoDriver> URRealTimeDriver::getDigitIoDriver(int deviceId) {
    if (deviceId == 0)
        return m_digitOutput;
    if (deviceId == 1)
        return m_digitInput;
    return nullptr;
}

void URRealTimeDriver::attach(const std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
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

bool URRealTimeDriver::start() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_urDriver) {
        COBOT_LOG.info() << "Already start, if want restart, stop first";
        return false;
    }
    m_urDriver = new CobotUrDriver(m_urMessage, m_attr_robot_ip.c_str());
    connect(m_urDriver, &CobotUrDriver::driverStartSuccess, this, &URRealTimeDriver::handleDriverReady);
    connect(m_urDriver, &CobotUrDriver::driverStartFailed, this, &URRealTimeDriver::handleDriverDisconnect);
    connect(m_urDriver, &CobotUrDriver::driverStopped, this, &URRealTimeDriver::handleDriverDisconnect);
    connect(m_urDriver, &QObject::destroyed, this, &URRealTimeDriver::handleObjectDestroy);
    m_urDriver->setServojTime(m_attr_servoj_time);
    m_urDriver->setServojLookahead(m_attr_servoj_lookahead);
    m_urDriver->setServojGain(m_attr_servoj_gain);
    m_urDriver->startDriver();

    // 这里是数字驱动的部分
    m_digitInput->setUrRealTimeCtrl(m_urDriver->m_urRealTimeCommCtrl);
    m_digitOutput->setUrRealTimeCtrl(m_urDriver->m_urRealTimeCommCtrl);
    m_digitInput->m_isInput = true;
    m_digitOutput->m_isOutput = true;
    return true;
}

void URRealTimeDriver::stop() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_urDriver) {
        m_curReqQValid = false;
        m_curReqQ.clear();
        m_isStarted = false;
        m_urDriver->stopDriver();
        m_urDriver->deleteLater();
        m_urDriver = nullptr;
        m_digitInput->setUrRealTimeCtrl(nullptr);
        m_digitOutput->setUrRealTimeCtrl(nullptr);
    }
}

bool URRealTimeDriver::setup(const QString& configFilePath) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    auto success = _setup(configFilePath);

    if (success) {
    } else {
        m_observers.clear(); // detach all observer
    }
    return success;
}

void URRealTimeDriver::robotStatusWatcher() {
    std::mutex m;
    std::unique_lock<std::mutex> uniqueLock(m);

    auto time_cur = std::chrono::high_resolution_clock::now();

    auto pStatus = std::make_shared<ArmRobotStatus>();
    std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > observer_tmp;
    std::vector<double> q_next;

    std::vector<double> daemonQ(6, 0);
    std::mutex daemonLock;
    auto daemonStatus = std::make_shared<ArmRobotStatus>();

    COBOT_LOG.notice("UrDriver") << "Watcher is Running.";
    std::thread servoDaemon([&]() { // 这个线程主要用于优化路径
        auto timeWait = std::chrono::high_resolution_clock::now();
        while (m_isWatcherRunning) {
            daemonLock.lock();
            if (m_mutex.try_lock()) {
                if (m_jointTargetFilter) {
                    m_jointTargetFilter->applyFilter(daemonQ, daemonStatus);
                }
                if (m_isStarted && m_urDriver) {
                    m_urDriver->servoj(daemonQ);
                }
                m_mutex.unlock();
            }
            daemonLock.unlock();
            timeWait += std::chrono::milliseconds(2);
            std::this_thread::sleep_until(timeWait);
        }
    });

    while (m_isWatcherRunning) {
        // Here Wait Ur Status Update
        m_urMessage->wait(uniqueLock);

        // Update Io Status when Robot Joint Update.
        if (m_mutex.try_lock()) {
            _updateDigitIoStatus();
            m_mutex.unlock();
        }

        // 计算时间间隔
        auto time_rdy = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_diff = time_rdy - time_cur; // 时间间隙
        time_cur = time_rdy;
        if (time_diff.count() > 0.012|| time_diff.count() < 0.006) {
//            COBOT_LOG.warning() << "Ur Status Updated Time Exception: " << time_diff.count();
        }

        // 抓取当前姿态
        if (m_mutex.try_lock()) {
            if (m_urDriver) {
                auto robotState = m_urDriver->m_urRealTimeCommCtrl->ur->getRobotState();
                q_next = robotState->getQActual();
                m_robotJointQCache = q_next;

                pStatus->q_actual = robotState->getQActual();
                pStatus->qd_actual = robotState->getQdActual();
            }
            m_mutex.unlock();
        }

        // 通知所有观察者，机器人数据已经更新。
        if (m_isStarted) {
            notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
                observer->onArmRobotStatusUpdate(pStatus);
            });
        }

        // 获取当前控制数据
        if (m_mutex.try_lock()) {
            if (m_curReqQValid) {
                if (m_curReqQ.size()) {
                    q_next = m_curReqQ;
                }
            }
            if (m_jointTargetFilter) { // 应用数据过滤，平滑运动
                m_jointTargetFilter->applyFilter(q_next, pStatus);
            }
            m_mutex.unlock();
        }

        // 更新控制驱动数据, 如果没有数据，默认会是当前状态。
//        if (m_isStarted && q_next.size() >= CobotUr::JOINT_NUM_) {
//            if (m_mutex.try_lock()) {
//                m_urDriver->servoj(q_next);
//                m_mutex.unlock();
//            }
//        }
        daemonLock.lock();
        daemonQ = q_next;
        *daemonStatus = *pStatus;
        daemonLock.unlock();
    }

    servoDaemon.join();
    COBOT_LOG.notice("UrDriver") << "Watcher shutdown!";
}

bool URRealTimeDriver::_setup(const QString& configFilePath) {
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_attr_robot_ip = json["robot_ip"].toString("localhost").toStdString();
        m_attr_servoj_time = json["servoj_time"].toDouble(0.08);
        m_attr_servoj_lookahead = json["servoj_lookahead"].toDouble(0.05);
        m_attr_servoj_gain = json["servoj_gain"].toDouble(300);

        m_isWatcherRunning = true;
        m_thread = std::thread(&URRealTimeDriver::robotStatusWatcher, this);
        return true;
    }
    return false;
}

void URRealTimeDriver::handleDriverReady() {
    m_isStarted = true;
    notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
        observer->onArmRobotConnect();
    });
}

QString URRealTimeDriver::getRobotUrl() {
    return m_attr_robot_ip.c_str();
}

void URRealTimeDriver::handleDriverDisconnect() {
    COBOT_LOG.info() << "URRealTimeDriver Disconnect";
    stop();
    notify([=](std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer) {
        observer->onArmRobotDisconnect();
    });
}

void URRealTimeDriver::notify(std::function<void(std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer)> func) {
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

void URRealTimeDriver::_updateDigitIoStatus() {
    if (m_digitInput && m_isStarted && m_urDriver) {
        auto outBits = m_urDriver->m_urCommCtrl->ur->getRobotState()->getDigitalOutputBits();
        auto inBits = m_urDriver->m_urCommCtrl->ur->getRobotState()->getDigitalInputBits();

        m_digitInput->m_inputIoStatus = inBits;
        m_digitOutput->m_outputIoStatus = outBits;

        m_digitInput->debugIoStatus();
        m_digitOutput->debugIoStatus();
    }
}

void URRealTimeDriver::clearAttachedObject() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_observers.clear();
    m_jointTargetFilter.reset();
}

std::vector<double> URRealTimeDriver::getRobotJointQ() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    return m_robotJointQCache;
}

void URRealTimeDriver::handleObjectDestroy(QObject* object) {
    if (*m_objectAlive) {
        handleDriverDisconnect();
    }
}

bool URRealTimeDriver::setTargetJointFilter(const std::shared_ptr<ArmRobotJointTargetFilter>& filter) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_jointTargetFilter = filter;
    return true;
}
