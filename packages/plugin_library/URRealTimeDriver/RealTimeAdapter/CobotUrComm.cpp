//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtNetwork/QHostAddress>
#include "CobotUrComm.h"
#include "CobotUrFirmwareQueryer.h"

CobotUrComm::CobotUrComm(std::condition_variable& cond_msg, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){

    m_robotState = std::make_shared<RobotState>(m_msg_cond);

    m_secSOCK = new QTcpSocket(this);

    m_secSOCK->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_secSOCK, &QTcpSocket::readyRead, this, &CobotUrComm::secReadData);
    connect(m_secSOCK, &QTcpSocket::connected, this, &CobotUrComm::secConnectHandle);
    connect(m_secSOCK, &QTcpSocket::disconnected, this, &CobotUrComm::secDisconnectHandle);
}

CobotUrComm::~CobotUrComm(){
    m_secSOCK->close();
}

void CobotUrComm::start(){
    if (m_host.isEmpty()) return;

    COBOT_LOG.info() << "Acquire firmware version: Connecting...";
    CobotUrFirmwareQueryer firmQueryer(m_host);
    firmQueryer.getVersion(m_robotState);

    /**
     * Link ur general message channel
     */
    m_secSOCK->connectToHost(m_host, 30002);
}

void CobotUrComm::setupHost(const QString& host){
    m_host = host;
}

void CobotUrComm::stop(){
}

void CobotUrComm::secReadData(){
    auto ba = m_secSOCK->read(2048);
    if (ba.size() > 0) {
        m_robotState->unpack((uint8_t*) ba.constData(), ba.size());
//        COBOT_LOG.info() << "Arrival UR Communication: " << ba.size();
    } else {
        m_robotState->setDisconnected();
        m_secSOCK->close();
    }
}

void CobotUrComm::secConnectHandle(){
    COBOT_LOG.info() << "Secondary interface: Got connection";
    localIp_ = m_secSOCK->localAddress().toString().toStdString();
    Q_EMIT connected();
}

std::string CobotUrComm::getLocalIp(){
    return localIp_;
}

void CobotUrComm::secDisconnectHandle(){
    COBOT_LOG.info() << "Secondary interface: disconnected";
    Q_EMIT disconnected();
}

void CobotUrComm::writeLine(const QByteArray& ba){
}
