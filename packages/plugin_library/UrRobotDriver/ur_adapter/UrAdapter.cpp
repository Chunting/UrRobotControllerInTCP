//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "UrAdapter.h"

UrAdapter::UrAdapter(){
}

UrAdapter::~UrAdapter(){
}

void UrAdapter::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal){
}

void UrAdapter::directControl(const std::string& command){
}

std::shared_ptr<QIODevice> UrAdapter::getSerialIoDevice(int devicdId){
    return std::shared_ptr<QIODevice>();
}

std::shared_ptr<AbstractDigitIoDriver> UrAdapter::getDigitIoDriver(int deviceId){
    return std::shared_ptr<AbstractDigitIoDriver>();
}

bool UrAdapter::getMoveData(uint32_t moveId, cv::Point3d& pos, cv::Vec3d& normal){
    return false;
}

void UrAdapter::clearMoveDataHistory(){
}

bool UrAdapter::setup(const QString& configFilePath){
    if (m_urDriver) {
        m_urDriver->halt();
    }

    std::string robot_ip = "localhost";
    m_urDriver = std::make_shared<UrDriver>(m_rt_msg_cond, m_msg_cond, robot_ip);
    m_urWatcher = std::make_shared<UrStatusWatcher>("rt", m_rt_msg_cond);
    m_urWatcher->start();
    return true;
}

void UrAdapter::start(){
    if (m_urDriver)
        m_urDriver->start();
}

void UrAdapter::pause(){
}

void UrAdapter::attach(std::shared_ptr<RobotStatusObserver> observer){
    for (auto& o : m_observerArray) {
        if (o == observer)
            return;
    }

    if (observer) {
        m_observerArray.push_back(observer);
    }
}
