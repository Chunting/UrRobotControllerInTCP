//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotMotomanUDPComm.h"
#include <QNetworkInterface>
CobotMotomanUDPComm::CobotMotomanUDPComm(std::condition_variable& cond_msg, const QString& robotIp, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){

    m_robotState = std::make_shared<MotomanRobotState>(m_msg_cond);
    m_robotIp = robotIp;
    m_udpSocket = new QUdpSocket(this);
    //m_udpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_udpSocket, &QUdpSocket::connected, this, &CobotMotomanUDPComm::onConnected);
    connect(m_udpSocket, &QUdpSocket::disconnected, this, &CobotMotomanUDPComm::onUDPDisconnect);
    connect(m_udpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanUDPComm::onSocketError);
}

void CobotMotomanUDPComm::onConnected(){
    COBOT_LOG.info() << "Motoman UDP sockect connected";
    Q_EMIT connected();
}

CobotMotomanUDPComm::~CobotMotomanUDPComm(){
    if (m_udpSocket) {
        m_udpSocket->close();
    }
}

void CobotMotomanUDPComm::start(){
    if (!m_udpSocket){
        COBOT_LOG.error() << "UDP Socket is NULL.";
        return;
    }
//    //获取本地IP.
//    foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
//    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
//        m_localIp=address.toString();
//    }

    //QHostAddress *robotAddress = new QHostAddress(m_robotIp);
    QHostAddress *localAddress = new QHostAddress(m_localIp);

    //m_udpSocket->abort();

    //COBOT_LOG.debug()<<"robotAddress->toString()"<<robotAddress->toString();
    //COBOT_LOG.debug()<<"QHostAddress::LocalHost"<<QHostAddress::LocalHost;
    m_udpSocket->bind(UDP_PORT);
    connect(m_udpSocket,&QUdpSocket::readyRead,this,&CobotMotomanUDPComm::readData);
    //m_udpSocket->connectToHost(m_robotIp,UDP_PORT);

    COBOT_LOG.notice()<<"Robot IP:"<<m_robotIp<<" UDP Port:"<<UDP_PORT;
//    if (!m_udpSocket->waitForConnected()) {
//        COBOT_LOG.error() << "Failed to connect the udp port of Motoman Robot. IP:"<<m_robotIp<<" udp port:"<<UDP_PORT;
//        return;
//    }
}

void CobotMotomanUDPComm::readData(){
    COBOT_LOG.debug()<<"Received UDP package";
    const int RECV_FRAME_LENGTH_=82;
    auto ba = m_udpSocket->readAll();

    COBOT_LOG.debug()<<"Received UDP package:"<<QString(ba.toHex());
    m_robotState->unpack(ba);
}

void CobotMotomanUDPComm::onUDPDisconnect(){
    COBOT_LOG.info() << "UDP Comm Disconnected !!!";
    m_udpSocket->close();
    m_udpSocket->deleteLater();
    m_udpSocket = nullptr;
    Q_EMIT disconnected();
}

void CobotMotomanUDPComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanUDPComm: " << m_udpSocket->errorString();
    Q_EMIT connectFail();
}


void CobotMotomanUDPCommCtrl::onUDPConnected() {

}
