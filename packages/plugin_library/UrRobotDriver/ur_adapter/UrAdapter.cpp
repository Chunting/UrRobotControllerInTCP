//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <extra2.h>
#include "UrAdapter.h"

UrAdapter::UrAdapter(){
    m_isStarted = false;
}

UrAdapter::~UrAdapter(){
}

bool UrAdapter::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal){
    return false;
}

void UrAdapter::move(uint32_t moveId, const std::vector<double>& jointPos){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);

    m_urDriver->servoj(jointPos);
}

void UrAdapter::directControl(const std::string& command){
}

std::shared_ptr<QIODevice> UrAdapter::getSerialIoDevice(int devicdId){
    return std::shared_ptr<QIODevice>();
}

std::shared_ptr<AbstractDigitIoDriver> UrAdapter::getDigitIoDriver(int deviceId){
    return std::shared_ptr<AbstractDigitIoDriver>();
}

bool UrAdapter::getMoveData(uint32_t moveId, std::vector<double>& moveData, int& moveDataType){
    return false;
}

void UrAdapter::clearMoveDataHistory(){
}

bool UrAdapter::setup(const QString& configFilePath){
    if (m_urDriver) {
        m_urDriver->halt();
    }

    QJsonObject jsonObject;
    std::string robot_ip = "localhost";
    if (loadJson(jsonObject, configFilePath)) {
        if (jsonObject.contains("robot_ip")) {
            robot_ip = jsonObject["robot_ip"].toString().toStdString();
        } else return false;

        m_urDriver = std::make_shared<UrDriver>(m_rt_msg_cond, m_msg_cond, robot_ip);
        m_urDriver->setServojTime(jsonObject["servoj_time"].toDouble(0.08));
        m_urDriver->setServojLookahead(jsonObject["servoj_lookahead"].toDouble(0.05));
        m_urWatcher = std::make_shared<UrStatusWatcher>(*this, "rt", m_rt_msg_cond);
        m_urWatcher->start();
        return true;
    }
    return false;
}

bool UrAdapter::start(){
    if (m_urDriver) {
        if (m_urDriver->start()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            if (m_urDriver->uploadProg()) {
                m_isStarted = true;
                COBOT_LOG.info() << "prog upload success";
                return true;
            }
        }
    }
    return false;
}

void UrAdapter::pause(){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);
    if (m_urDriver) {
        m_isStarted = false;
        m_urDriver->closeServo({});
        m_urDriver->stopTraj();
        m_urDriver->halt();
    }
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

void UrAdapter::notify(std::function<void(std::shared_ptr<RobotStatusObserver>&)> applyFunc){
    if (applyFunc) {
        for (auto& o : m_observerArray) {
            applyFunc(o);
        }
    }
}
