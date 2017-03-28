//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonObject>
#include <extra2.h>
#include "URRealTimeDriver.h"


URRealTimeDriver::URRealTimeDriver(){
    m_isWatcherRunning = false;
}

URRealTimeDriver::~URRealTimeDriver(){
    if (m_isWatcherRunning) {
        m_isWatcherRunning = false;
        m_rt_msg_cond.notify_all();
        m_thread.join();
    }
}

void URRealTimeDriver::move(const std::vector<double>& q){
}

std::shared_ptr<AbstractDigitIoDriver> URRealTimeDriver::getDigitIoDriver(int deviceId){
    return nullptr;
}

void URRealTimeDriver::attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer){
    std::lock_guard<std::mutex> lock_guard(m_mutex);

    for (auto& iter : m_observers) {
        if (iter.get() == observer.get()) {
            return; // Already have attached
        }
    }

    if (observer) {
        m_observers.push_back(observer);
    }
}

bool URRealTimeDriver::start(){
    if (m_urDriver) {
        return m_urDriver->start();
    }
    return false;
}

void URRealTimeDriver::stop(){
}

bool URRealTimeDriver::setup(const QString& configFilePath){
    std::lock_guard<std::mutex> lock_guard(m_mutex);
    auto success = _setup(configFilePath);

    if (!success) {
        m_observers.clear(); // detach all observer
    }
    return success;
}

void URRealTimeDriver::robotStatusWatcher(){
    std::mutex m;
    std::unique_lock<std::mutex> lck(m);

    auto time_cur = std::chrono::high_resolution_clock::now();

    auto pStatus = std::make_shared<ArmRobotStatus>();
    std::vector<std::shared_ptr<ArmRobotRealTimeStatusObserver> > observer_tmp;
    while (m_isWatcherRunning) {
        m_rt_msg_cond.wait(lck);

        auto time_rdy = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_diff = time_rdy - time_cur;
        time_cur = time_rdy;

        pStatus->q_actual = m_urDriver->rt_interface_->robot_state_->getQActual();

//        COBOT_LOG.info() << "Status Updated: " << time_diff.count();


        // Notify all attached observer
        if (m_mutex.try_lock()) {
            observer_tmp = m_observers;
            m_mutex.unlock();
        }
        for (auto& ob : observer_tmp) {
            ob->onArmRobotStatusUpdate(pStatus);
        }
    }
}

bool URRealTimeDriver::_setup(const QString& configFilePath){
    if (m_urDriver)
        return false;

    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_attr_robot_ip = json["robot_ip"].toString("localhost").toStdString();
        m_attr_servoj_time = json["servoj_time"].toDouble(0.08);
        m_attr_servoj_lookahead = json["servoj_lookahead"].toDouble(0.05);
        m_attr_servoj_gain = json["servoj_gain"].toDouble(300);

        bool ur_create_success = false;
        try {
            m_urDriver = std::make_shared<UrDriver>(m_rt_msg_cond, m_msg_cond, m_attr_robot_ip);
            m_urDriver->setServojTime(m_attr_servoj_time);
            m_urDriver->setServojLookahead(m_attr_servoj_lookahead);
            m_urDriver->setServojGain(m_attr_servoj_gain);

            m_isWatcherRunning = true;
            m_thread = std::thread(&URRealTimeDriver::robotStatusWatcher, this);
            ur_create_success = true;
        } catch (std::exception& e) {
        }
        return ur_create_success;
    }
    return false;
}
