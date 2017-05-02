//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotMotomanRealTimeComm.h"
#include <cobotsys.h>

CobotMotomanRealTimeComm::CobotMotomanRealTimeComm(std::condition_variable& cond_msg, const QString& robotIp, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){
    m_robotState = std::make_shared<RobotState>(m_msg_cond);
    m_tcpServer = new QTcpServer(this);
    m_robotIp = robotIp;
    m_SOCKET = new QTcpSocket(this);
    m_SOCKET->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_SOCKET, &QTcpSocket::connected, this, &CobotMotomanRealTimeComm::onConnected);
    connect(m_SOCKET, &QTcpSocket::disconnected, this, &CobotMotomanRealTimeComm::onDisconnected);
    connect(m_SOCKET, &QTcpSocket::readyRead, this, &CobotMotomanRealTimeComm::readData);

    connect(m_tcpServer, &QTcpServer::newConnection, this, &CobotMotomanRealTimeComm::motomanProgConnect);



    connect(m_SOCKET, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanRealTimeComm::onSocketError);

    m_rtSOCKET = nullptr;
    keepalive = 1;
}

void CobotMotomanRealTimeComm::onConnected(){
    COBOT_LOG.info() << "RealTime Connection Ready.";
    Q_EMIT connected();
}

void CobotMotomanRealTimeComm::onDisconnected(){
    COBOT_LOG.info() << "CobotMotomanRealTimeComm real time disconnected";
    Q_EMIT disconnected();
}

CobotMotomanRealTimeComm::~CobotMotomanRealTimeComm(){
    if (m_rtSOCKET) {
        m_rtSOCKET->close();
    }
    if (m_SOCKET) {
        m_SOCKET->close();
    }
}

void CobotMotomanRealTimeComm::start(){
    m_SOCKET->connectToHost(m_robotIp, CobotMotoman::TCP_PORT);
    m_tcpServer->listen(QHostAddress::AnyIPv4, REVERSE_PORT_);
}

void CobotMotomanRealTimeComm::readData(){
    auto ba = m_SOCKET->read(2048);
//    if (ba.size()) {
//        m_robotState->unpack((uint8_t*) ba.constData());
//    }

}

void CobotMotomanRealTimeComm::writeLine(const QByteArray& ba){
    auto nba = ba;
    if (nba.size()) {
        if (nba.at(nba.size() - 1) != '\n') {
            nba.push_back('\n');
        }
        m_SOCKET->write(nba);
    }
}

void CobotMotomanRealTimeComm::motomanProgConnect(){
    if (m_rtSOCKET)
        return;

    m_rtSOCKET = m_tcpServer->nextPendingConnection();
    m_rtSOCKET->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    connect(m_rtSOCKET, &QTcpSocket::disconnected, this, &CobotMotomanRealTimeComm::onRealTimeDisconnect);
    Q_EMIT realTimeProgConnected();
}

void CobotMotomanRealTimeComm::servoj(const std::vector<double>& j){
    if (m_rtSOCKET == nullptr) {
        return;
    }

    auto positions = j;
    if (positions.size() != 6) {
        positions = m_robotState->getQActual();
    }
//
//    unsigned int bytes_written;
//    int tmp;
//    unsigned char buf[28];
//    for (int i = 0; i < 6; i++) {
//        tmp = htonl((int) (positions[i] * MULT_JOINTSTATE_));
//        buf[i * 4] = tmp & 0xff;
//        buf[i * 4 + 1] = (tmp >> 8) & 0xff;
//        buf[i * 4 + 2] = (tmp >> 16) & 0xff;
//        buf[i * 4 + 3] = (tmp >> 24) & 0xff;
//    }
//    tmp = htonl((int) keepalive);
//    buf[6 * 4] = tmp & 0xff;
//    buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
//    buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
//    buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
//    bytes_written = m_rtSOCKET->write((char*) buf, 28);
}

void CobotMotomanRealTimeComm::stopProg(){
    if (keepalive) {
        keepalive = 0;
        COBOT_LOG.info() << "Stopping Motoman Driver Program";
        servoj({});

        /**
         * 如果默认的程序不能正确的停止，则需要下面的语句。
         */
        //   m_SOCKET->write("stopj(10)\n");
    }
}

void CobotMotomanRealTimeComm::onRealTimeDisconnect(){
    COBOT_LOG.info() << "RealTime Ctrl Disconnected !!!";

    m_rtSOCKET->close();
    m_rtSOCKET->deleteLater();
    m_rtSOCKET = nullptr;

    Q_EMIT realTimeProgDisconnect();
}



void CobotMotomanRealTimeComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanRealTimeComm: " << m_SOCKET->errorString();
    Q_EMIT connectFail();
}




