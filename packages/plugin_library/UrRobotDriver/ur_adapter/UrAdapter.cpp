//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <extra2.h>
#include "UrAdapter.h"

UrAdapter::UrAdapter() : QObject(nullptr){
    m_isStarted = false;
    m_pConnectionCheckTimer = new QTimer(this);
    m_pConnectionCheckTimer->setInterval(100);
    connect(m_pConnectionCheckTimer, &QTimer::timeout, this, &UrAdapter::tickCheckService);

    m_connectionNotifyStatus = true;
    m_disconnectNotifyStatus = false;


    m_pConnectionCheckTimer->start();
    m_onceStartCall = true;
}

UrAdapter::~UrAdapter(){
}

bool UrAdapter::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);

    return false;
}

void UrAdapter::move(uint32_t moveId, const std::vector<double>& jointPos){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);

    if (jointPos.size() == 6) {
        m_urDriver->servoj(jointPos);
    }
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
    std::lock_guard<std::mutex> lock_guard(m_resMutex);

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
        m_urDriver->setServojGain(jsonObject["servoj_gain"].toDouble(300));

        m_urWatcher = std::make_shared<UrStatusWatcher>(*this, "rt", m_rt_msg_cond);
        m_urWatcher->start();
        return true;
    }
    return false;
}

bool UrAdapter::start(){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);

    if (m_isStarted)
        return true;

    if (m_urDriver) {
        if (m_onceStartCall) {
            m_onceStartCall = false;
            m_urDriver->start();
        }

        connectedOnceSetup();
        return true;
    }
    return false;
}

void UrAdapter::pause(){
    std::lock_guard<std::mutex> lock_guard(m_resMutex);
    if (m_urDriver && m_isStarted) {
        m_isStarted = false;
        m_urDriver->closeServo({});
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
    if (applyFunc && m_isStarted) {
        for (auto& o : m_observerArray) {
            applyFunc(o);
        }
    }
}

void UrAdapter::tickCheckService(){
    if (m_urDriver) {
        if (m_urDriver->rt_interface_->connected_) {
            if (m_connectionNotifyStatus) {
                if (connectedOnceSetup()) {
                    notify([=](std::shared_ptr<RobotStatusObserver>& o){
                        o->onRobotConnected(std::dynamic_pointer_cast<AbstractRobotDriver>(shared_from_this()));
                    });
                    m_connectionNotifyStatus = false;
                    m_disconnectNotifyStatus = true;
                }
            }
        } else if (m_disconnectNotifyStatus) {
            notify([=](std::shared_ptr<RobotStatusObserver>& o){
                o->onRobotDisconnected(std::dynamic_pointer_cast<AbstractRobotDriver>(shared_from_this()));
            });
            m_connectionNotifyStatus = true;
            m_disconnectNotifyStatus = false;
        }
    }
}

bool UrAdapter::connectedOnceSetup(){
    if (m_urDriver) {
        if (m_urDriver->rt_interface_->connected_) {
            if (m_urDriver->uploadProg()) {
                m_isStarted = true;
                COBOT_LOG.info() << "prog upload success";
                return true;
            }
        }
    }
    return false;
}

bool UrAdapter::isReady() const{
    return m_isStarted;
}
