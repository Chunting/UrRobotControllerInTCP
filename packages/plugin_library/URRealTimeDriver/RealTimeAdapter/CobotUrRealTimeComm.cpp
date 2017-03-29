//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotUrRealTimeComm.h"


CobotUrRealTimeComm::CobotUrRealTimeComm(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){
    m_robotState = std::make_shared<RobotStateRT>(m_msg_cond);
    m_tcpServer = new QTcpServer(this);
    m_hostIp = hostIp;
    m_SOCKET = new QTcpSocket(this);
    m_SOCKET->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_SOCKET, &QTcpSocket::connected, this, &CobotUrRealTimeComm::onConnected);
    connect(m_SOCKET, &QTcpSocket::disconnected, this, &CobotUrRealTimeComm::onDisconnected);
    connect(m_SOCKET, &QTcpSocket::readyRead, this, &CobotUrRealTimeComm::readData);

    connect(m_tcpServer, &QTcpServer::newConnection, this, &CobotUrRealTimeComm::urProgConnect);
    m_rtSOCKET = nullptr;
}

void CobotUrRealTimeComm::onConnected(){
    COBOT_LOG.info() << "RealTime Connection Ready.";
    Q_EMIT connected();
}

void CobotUrRealTimeComm::onDisconnected(){
    COBOT_LOG.info() << "CobotUrRealTimeComm real time disconnected";
    Q_EMIT disconnected();
}

CobotUrRealTimeComm::~CobotUrRealTimeComm(){
    if (m_rtSOCKET) {
        servoj({}, 0);
        m_rtSOCKET->close();
    }
    m_SOCKET->close();
}

void CobotUrRealTimeComm::start(){
    m_SOCKET->connectToHost(m_hostIp, 30003);
    m_tcpServer->listen(QHostAddress::AnyIPv4, REVERSE_PORT_);
}

void CobotUrRealTimeComm::readData(){
    auto ba = m_SOCKET->read(2048);
    if (ba.size()) {
        m_robotState->unpack((uint8_t*) ba.constData());

        servoj(m_robotState->getQActual());
    }
}

void CobotUrRealTimeComm::writeLine(const QByteArray& ba){
    m_SOCKET->write(ba);
}

void CobotUrRealTimeComm::urProgConnect(){
    if (m_rtSOCKET)
        return;

    m_rtSOCKET = m_tcpServer->nextPendingConnection();
    Q_EMIT realTimeProgConnected();
}

void CobotUrRealTimeComm::servoj(const std::vector<double>& j, int keepalive){
    if (m_rtSOCKET == nullptr) {
        COBOT_LOG.error() << "No rt connection";
        return;;
    }
    auto positions = j;
    if (positions.size() != 6) {
        positions.resize(6, 0);
    }

    unsigned int bytes_written;
    int tmp;
    unsigned char buf[28];
    for (int i = 0; i < 6; i++) {
        tmp = htonl((int) (positions[i] * MULT_JOINTSTATE_));
        buf[i * 4] = tmp & 0xff;
        buf[i * 4 + 1] = (tmp >> 8) & 0xff;
        buf[i * 4 + 2] = (tmp >> 16) & 0xff;
        buf[i * 4 + 3] = (tmp >> 24) & 0xff;
    }
    tmp = htonl((int) keepalive);
    buf[6 * 4] = tmp & 0xff;
    buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
    buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
    buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
    bytes_written = m_rtSOCKET->write((char*) buf, 28);
    if (keepalive == 0) {
        COBOT_LOG.info() << "Try to close ur";
    }
}


