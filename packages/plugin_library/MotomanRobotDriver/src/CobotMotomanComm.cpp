//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtNetwork/QHostAddress>
#include "CobotMotomanComm.h"
#include "CobotMotomanFirmwareQueryer.h"
#include <cobotsys.h>

CobotMotomanComm::CobotMotomanComm(std::condition_variable& cond_msg, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg),m_cmdID(0){

    m_robotState = std::make_shared<RobotState>(m_msg_cond);

    m_tcpSocket = new QTcpSocket(this);

    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &CobotMotomanComm::processData);
    connect(m_tcpSocket, &QTcpSocket::connected, this, &CobotMotomanComm::secConnectHandle);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &CobotMotomanComm::secDisconnectHandle);
    connect(m_tcpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanComm::onSocketError);
}

CobotMotomanComm::~CobotMotomanComm(){
    m_tcpSocket->close();
}

void CobotMotomanComm::start(){
    if (m_host.isEmpty()) return;

    COBOT_LOG.info() << "Acquire firmware version: Connecting...";
    CobotMotomanFirmwareQueryer firmQueryer(m_host);
    if (firmQueryer.getVersion(m_robotState)) {
        /**
         * Link motoman general message channel
         */
        //TODO Here
        m_tcpSocket->connectToHost(m_host, CobotMotoman::TCP_PORT);//Motoman tcp port ID is 11000
    } else {
        Q_EMIT connectFail();
    }
}

void CobotMotomanComm::setupHost(const QString& host){
    m_host = host;
}

void CobotMotomanComm::stop(){
}

void CobotMotomanComm::processData(){
    auto ba = m_tcpSocket->read(2048);
    if (ba.size() > 0) {
        m_robotState->unpack((uint8_t*) ba.constData(), ba.size());
    } else {
        m_robotState->setDisconnected();
        m_tcpSocket->close();
    }
}

void CobotMotomanComm::secConnectHandle(){
    COBOT_LOG.info() << "Secondary interface: Got connection";
    localIp_ = m_tcpSocket->localAddress().toString().toStdString();
    Q_EMIT connected();
}

std::string CobotMotomanComm::getLocalIp(){
    return localIp_;
}

void CobotMotomanComm::secDisconnectHandle(){
    COBOT_LOG.info() << "Secondary interface: disconnected";
    Q_EMIT disconnected();
}

void CobotMotomanComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanComm: " << m_tcpSocket->errorString();
    Q_EMIT connectFail();
}
void CobotMotomanComm::executeCmd(const CobotMotoman::ROBOTCMD CmdID) {
    QByteArray cmd;
    cmd.resize(CobotMotoman::FRAME_LENGTH_);
    QByteArray IP=IntToArray(m_tcpSocket->localAddress().toIPv4Address());
    switch (CmdID){
        case CobotMotoman::CMD_START_UDP:
            cmd[ 0] = 0xc0;
            cmd[ 1] = IP[0];
            cmd[ 2] = IP[1];
            cmd[ 3] = IP[2];
            cmd[ 4] = IP[3];
            break;
        case CobotMotoman::CMD_SERVO_ON:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0xff;
            cmd[ 3] = 0x00;
            break;
        case CobotMotoman::CMD_SERVO_OFF:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0x00;
            cmd[ 3] = 0x00;
            break;
        default:
            COBOT_LOG.error()<<"Undefined command.";
    }
    sendCmd(cmd);
}

void CobotMotomanComm::sendCmd(QByteArray &cmd) {
    if(cmd.size()!=CobotMotoman::FRAME_LENGTH_){
        COBOT_LOG.error()<<"The size of tcp frame length is not "<<CobotMotoman::FRAME_LENGTH_;
        return;
    }
    if(m_cmdID<255){
        m_cmdID++;
    }else {
        m_cmdID = 0;
    }
    cmd[CobotMotoman::FRAME_LENGTH_-2]=m_cmdID;
    cmd[CobotMotoman::FRAME_LENGTH_-1] = 0xff;
    if(m_tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        m_tcpSocket->write(cmd); //write the data itself
    }
    else{
        COBOT_LOG.error()<<"TCP socket state is QAbstractSocket::UnconnectedState.";
        return;
    }

}