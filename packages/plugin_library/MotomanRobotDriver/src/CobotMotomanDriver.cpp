//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CobotMotomanDriver.h"
#include "CobotMotoman.h"

CobotMotomanDriver::CobotMotomanDriver(std::condition_variable& rt_msg_cond,
                             std::condition_variable& msg_cond,
                             const QString& robotAddr,
                             QObject* parent) : QObject(parent){
    m_motomanCommCtrl = new CobotMotomanCommCtrl(msg_cond, robotAddr, this);
    m_motomanRealTimeCommCtrl = new CobotMotomanRealTimeCommCtrl(rt_msg_cond, robotAddr, this);

    connect(m_motomanCommCtrl->motoman, &CobotMotomanComm::connected, this, &CobotMotomanDriver::handleCommConnected);
    connect(m_motomanRealTimeCommCtrl->motoman, &CobotMotomanRealTimeComm::connected, this, &CobotMotomanDriver::handleRTCommConnected);

    connect(m_motomanCommCtrl->motoman, &CobotMotomanComm::disconnected, this, &CobotMotomanDriver::handleDisconnected);
    connect(m_motomanRealTimeCommCtrl->motoman, &CobotMotomanRealTimeComm::disconnected, this, &CobotMotomanDriver::handleDisconnected);

    connect(m_motomanRealTimeCommCtrl->motoman, &CobotMotomanRealTimeComm::realTimeProgConnected,
            this, &CobotMotomanDriver::handleRTProgConnect);
    connect(m_motomanRealTimeCommCtrl->motoman, &CobotMotomanRealTimeComm::realTimeProgDisconnect,
            this, &CobotMotomanDriver::handleRTProgDisconnect);

    connect(m_motomanCommCtrl->motoman, &CobotMotomanComm::connectFail, this, &CobotMotomanDriver::handleDisconnected);
    connect(m_motomanRealTimeCommCtrl->motoman, &CobotMotomanRealTimeComm::connectFail, this, &CobotMotomanDriver::handleDisconnected);


    m_noDisconnectedAccept = false;

    servoj_gain_ = 300;
    servoj_lookahead_time_ = 0.05;
    servoj_time_ = 0.016;

    m_connectTime = 0;
    m_isConnected = false;
}

CobotMotomanDriver::~CobotMotomanDriver(){
    stopDriver();
}

void CobotMotomanDriver::handleCommConnected(){
    m_connectTime++;
    if (m_connectTime >= 2) {
        onConnectSuccess();
    }
}

void CobotMotomanDriver::handleRTCommConnected(){
    m_connectTime++;
    if (m_connectTime >= 2) {
        onConnectSuccess();
    }
}

void CobotMotomanDriver::startDriver(){
    m_noDisconnectedAccept = true;
    m_disconnectCount = 0;
    m_connectTime = 0;
    m_isConnected = false;
    m_motomanCommCtrl->startComm();
    m_motomanRealTimeCommCtrl->startComm();
}

void CobotMotomanDriver::handleDisconnected(){
    m_connectTime = 0;
    m_isConnected = false;
    m_disconnectCount++;

    if (m_noDisconnectedAccept) {
        Q_EMIT driverStartFailed();
    }
    if (m_disconnectCount >= 2) {
        Q_EMIT driverStopped();
    }
}

void CobotMotomanDriver::stopDriver(){
    m_noDisconnectedAccept = false;
    m_motomanRealTimeCommCtrl->requireStopServoj();
}


void CobotMotomanDriver::setServojTime(double t){
    if (t > 0.008) {
        servoj_time_ = t;
    } else {
        servoj_time_ = 0.008;
    }
}

void CobotMotomanDriver::setServojLookahead(double t){
    if (t > 0.03) {
        if (t < 0.2) {
            servoj_lookahead_time_ = t;
        } else {
            servoj_lookahead_time_ = 0.2;
        }
    } else {
        servoj_lookahead_time_ = 0.03;
    }
}

void CobotMotomanDriver::setServojGain(double g){
    if (g > 100) {
        if (g < 2000) {
            servoj_gain_ = g;
        } else {
            servoj_gain_ = 2000;
        }
    } else {
        servoj_gain_ = 100;
    }
}

void CobotMotomanDriver::onConnectSuccess(){
    m_isConnected = true;
    ip_addr_ = m_motomanCommCtrl->motoman->getLocalIp();
    COBOT_LOG.info() << "Local Ip: " << ip_addr_;
    m_motomanCommCtrl->motoman->executeCmd(CobotMotoman::CMD_SERVO_ON);
    m_motomanCommCtrl->motoman->executeCmd(CobotMotoman::CMD_START_UDP);
    Q_EMIT driverStartSuccess();
}

void CobotMotomanDriver::handleRTProgConnect(){
    COBOT_LOG.info() << "Prog Upload Success";
}

void CobotMotomanDriver::handleRTProgDisconnect(){
    if (m_isConnected) {
        Q_EMIT driverStopped();
    }
}

void CobotMotomanDriver::servoj(const std::vector<double>& positions){
    if (m_motomanRealTimeCommCtrl) {
        m_motomanRealTimeCommCtrl->motoman->asyncServoj(positions);
    }
}