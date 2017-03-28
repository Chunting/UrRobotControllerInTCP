//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotUrRealTimeComm.h"


CobotUrRealTimeComm::CobotUrRealTimeComm(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){
    m_robotState = std::make_shared<RobotStateRT>(m_msg_cond);
    m_hostIp = hostIp;
    m_SOCKET = new QTcpSocket(this);
    m_SOCKET->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_SOCKET, &QTcpSocket::connected, this, &CobotUrRealTimeComm::onConnected);
    connect(m_SOCKET, &QTcpSocket::disconnected, this, &CobotUrRealTimeComm::onDisconnected);
    connect(m_SOCKET, &QTcpSocket::readyRead, this, &CobotUrRealTimeComm::readData);
}

void CobotUrRealTimeComm::onConnected(){
}

void CobotUrRealTimeComm::onDisconnected(){
    COBOT_LOG.info() << "CobotUrRealTimeComm real time disconnected";
}

CobotUrRealTimeComm::~CobotUrRealTimeComm(){
    m_SOCKET->close();
}

void CobotUrRealTimeComm::start(){
    m_SOCKET->connectToHost(m_hostIp, 30003);
}

void CobotUrRealTimeComm::readData(){
    auto ba = m_SOCKET->read(2048);
    if (ba.size()) {
        m_robotState->unpack((uint8_t*) ba.constData());
    }
}

void CobotUrRealTimeComm::writeLine(const QByteArray& ba){
    m_SOCKET->write(ba);
}


