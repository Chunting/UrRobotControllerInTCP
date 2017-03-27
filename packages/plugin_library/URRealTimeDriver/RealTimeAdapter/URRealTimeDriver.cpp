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
    if (m_isWatcherRunning){
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
}

bool URRealTimeDriver::start(){
    return false;
}

void URRealTimeDriver::stop(){
}

bool URRealTimeDriver::setup(const QString& configFilePath){
    std::lock_guard<std::mutex> lock_guard(m_mutex);

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

void URRealTimeDriver::robotStatusWatcher(){
    std::mutex m;
    std::unique_lock<std::mutex> lck(m);

    auto time_cur = std::chrono::high_resolution_clock::now();
    while (m_isWatcherRunning) {
        m_rt_msg_cond.wait(lck);

        auto time_rdy = std::chrono::high_resolution_clock::now();
    }
}
