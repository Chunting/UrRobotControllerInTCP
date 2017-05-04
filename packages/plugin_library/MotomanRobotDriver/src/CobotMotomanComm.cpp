//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include "CobotMotoman.h"
#include "CobotMotomanComm.h"

CobotMotomanComm::CobotMotomanComm(std::condition_variable& udp_msg_cond,
                             std::condition_variable& tcp_msg_cond,
                             const QString& robotAddr,
                             QObject* parent) : QObject(parent){
    m_motomanTCPCommCtrl = new CobotMotomanTCPCommCtrl(tcp_msg_cond, robotAddr, this);
    m_motomanUDPCommCtrl = new CobotMotomanUDPCommCtrl(udp_msg_cond, robotAddr, this);

    connect(m_motomanTCPCommCtrl->motoman, &CobotMotomanTCPComm::connected, this, &CobotMotomanComm::handleTCPConnected);
    connect(m_motomanUDPCommCtrl->motoman, &CobotMotomanUDPComm::connected, this, &CobotMotomanComm::handleUDPConnected);

    connect(m_motomanTCPCommCtrl->motoman, &CobotMotomanTCPComm::disconnected, this, &CobotMotomanComm::handleTCPDisconnected);
    connect(m_motomanUDPCommCtrl->motoman, &CobotMotomanUDPComm::disconnected, this, &CobotMotomanComm::handleUDPDisconnected);

    connect(m_motomanTCPCommCtrl->motoman, &CobotMotomanTCPComm::connectFail, this, &CobotMotomanComm::handleTCPDisconnected);
    connect(m_motomanUDPCommCtrl->motoman, &CobotMotomanUDPComm::connectFail, this, &CobotMotomanComm::handleUDPDisconnected);


    m_noDisconnectedAccept = false;

    servoj_gain_ = 300;
    servoj_lookahead_time_ = 0.05;
    servoj_time_ = 0.016;

    m_connectTime = 0;
    m_isConnected = false;
}

CobotMotomanComm::~CobotMotomanComm(){
    stopDriver();
}

void CobotMotomanComm::handleTCPConnected(){
    m_connectTime++;
    if (m_connectTime >= 2) {
        onConnectSuccess();
    }
}
void CobotMotomanComm::handleTCPDisconnected(){
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

void CobotMotomanComm::handleUDPConnected(){
    COBOT_LOG.info() << "UDP Connected";
}

void CobotMotomanComm::handleUDPDisconnected(){
    if (m_isConnected) {
        Q_EMIT driverStopped();
    }
}
void CobotMotomanComm::startDriver(){
    m_noDisconnectedAccept = true;
    m_disconnectCount = 0;
    m_connectTime = 0;
    m_isConnected = false;
    m_motomanTCPCommCtrl->startComm();
    m_motomanUDPCommCtrl->startComm();
}


void CobotMotomanComm::stopDriver(){
    m_noDisconnectedAccept = false;
    m_motomanUDPCommCtrl->requireStopServoj();
}


void CobotMotomanComm::setServojTime(double t){
    if (t > 0.008) {
        servoj_time_ = t;
    } else {
        servoj_time_ = 0.008;
    }
}

void CobotMotomanComm::setServojLookahead(double t){
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

void CobotMotomanComm::setServojGain(double g){
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

void CobotMotomanComm::onConnectSuccess(){
    m_isConnected = true;
    ip_addr_ = m_motomanTCPCommCtrl->motoman->getLocalIp();
    COBOT_LOG.info() << "Local Ip: " << ip_addr_;
    m_motomanTCPCommCtrl->motoman->executeCmd(CobotMotomanTCPComm::CMD_SERVO_ON);
    m_motomanTCPCommCtrl->motoman->executeCmd(CobotMotomanTCPComm::CMD_START_UDP);
    Q_EMIT driverStartSuccess();
}

void CobotMotomanComm::servoj(const std::vector<double>& positions){
    if (m_motomanTCPCommCtrl) {
        m_motomanTCPCommCtrl->motoman->asyncServoj(positions);
    }
}