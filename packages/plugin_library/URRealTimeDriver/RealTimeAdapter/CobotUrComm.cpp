//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtNetwork/QHostAddress>
#include "CobotUrComm.h"
#include "CobotUrFirmwareQueryer.h"
#include <cobotsys.h>

CobotUrComm::CobotUrComm(std::condition_variable& cond_msg, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){

    m_robotState = std::make_shared<RobotState>(m_msg_cond);

    m_tcpSocket = new QTcpSocket(this);

    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &CobotUrComm::processData);
    connect(m_tcpSocket, &QTcpSocket::connected, this, &CobotUrComm::secConnectHandle);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &CobotUrComm::secDisconnectHandle);
    connect(m_tcpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotUrComm::onSocketError);
}

CobotUrComm::~CobotUrComm(){
    m_tcpSocket->close();
    INFO_DESTRUCTOR(this);
}

void CobotUrComm::start(){
    if (m_host.isEmpty()) return;

    COBOT_LOG.info() << "Acquire firmware version: Connecting...";
    CobotUrFirmwareQueryer firmQueryer(m_host);
    if (firmQueryer.getVersion(m_robotState)) {
        /**
         * Link ur general message channel
         */
        m_tcpSocket->connectToHost(m_host, 30002);
    } else {
        Q_EMIT connectFail();
    }
}

void CobotUrComm::setupHost(const QString& host){
    m_host = host;
}

void CobotUrComm::stop(){
}

void CobotUrComm::processData(){
    auto ba = m_tcpSocket->read(2048);
    if (ba.size() > 0) {
        m_robotState->unpack((uint8_t*) ba.constData(), ba.size());
    } else {
        m_robotState->setDisconnected();
        m_tcpSocket->close();
    }
}

void CobotUrComm::secConnectHandle(){
    COBOT_LOG.info() << "Secondary interface: Got connection";
    localIp_ = m_tcpSocket->localAddress().toString().toStdString();
    Q_EMIT connected();
}

std::string CobotUrComm::getLocalIp(){
    return localIp_;
}

void CobotUrComm::secDisconnectHandle(){
    COBOT_LOG.info() << "Secondary interface: disconnected";
    Q_EMIT disconnected();
}

void CobotUrComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotUrComm: " << m_tcpSocket->errorString();
    Q_EMIT connectFail();
}
