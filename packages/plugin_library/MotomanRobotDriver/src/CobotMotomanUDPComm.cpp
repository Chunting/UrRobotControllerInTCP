//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotMotomanUDPComm.h"

CobotMotomanUDPComm::CobotMotomanUDPComm(std::condition_variable& cond_msg, const QString& robotIp, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){

    m_robotState = std::make_shared<MotomanRobotState>(m_msg_cond);
    m_robotIp = robotIp;
    m_udpSocket = new QUdpSocket(this);
    //m_udpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_udpSocket, &QUdpSocket::connected, this, &CobotMotomanUDPComm::onConnected);
    connect(m_udpSocket, &QUdpSocket::disconnected, this, &CobotMotomanUDPComm::onRealTimeDisconnect);
    connect(m_udpSocket,&QUdpSocket::readyRead,this,&CobotMotomanUDPComm::readData);
    connect(m_udpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanUDPComm::onSocketError);
    keepalive = 1;
}

void CobotMotomanUDPComm::onConnected(){
    COBOT_LOG.info() << "RealTime Connection Ready.";
    Q_EMIT connected();
}

CobotMotomanUDPComm::~CobotMotomanUDPComm(){
    if (m_udpSocket) {
        m_udpSocket->close();
    }
}

void CobotMotomanUDPComm::start(){
    if (m_udpSocket)
        return;
    QHostAddress *robotAddress  = new QHostAddress(m_robotIp);
    m_udpSocket->abort();
    //bind and connect to host 哪个在前，哪个在后，有待确定。
    m_udpSocket->bind(*robotAddress, UDP_PORT);
    m_udpSocket->connectToHost(m_robotIp,UDP_PORT);
    if (!m_udpSocket->waitForConnected()) {
        COBOT_LOG.error() << "Failed to connect the udp port of Motoman Robot. IP:"<<m_robotIp<<" udp port:"<<UDP_PORT;
        return;
    }
    Q_EMIT realTimeProgConnected();
}

void CobotMotomanUDPComm::readData(){
    const int RECV_FRAME_LENGTH_=82;
    auto ba = m_udpSocket->readAll();
    m_robotState->unpack(ba);
}

void CobotMotomanUDPComm::onRealTimeDisconnect(){
    COBOT_LOG.info() << "RealTime Ctrl Disconnected !!!";
    m_udpSocket->close();
    m_udpSocket->deleteLater();
    m_udpSocket = nullptr;
    Q_EMIT disconnected();
}

void CobotMotomanUDPComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanRealTimeComm: " << m_udpSocket->errorString();
    Q_EMIT connectFail();
}




