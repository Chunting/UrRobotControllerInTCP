//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtNetwork/QHostAddress>
#include "CobotMotomanTCPComm.h"
#include "CobotMotomanFirmwareQueryer.h"

CobotMotomanTCPComm::CobotMotomanTCPComm(std::condition_variable& cond_msg, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg),m_cmdID(0),m_LastCmdID(CMD_SERVO_OFF){

    m_robotState = std::make_shared<MotomanRobotState>(m_msg_cond);

    m_tcpSocket = new QTcpSocket(this);

    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &CobotMotomanTCPComm::processData);
    connect(m_tcpSocket, &QTcpSocket::connected, this, &CobotMotomanTCPComm::secConnectHandle);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &CobotMotomanTCPComm::secDisconnectHandle);
    connect(m_tcpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanTCPComm::onSocketError);

    connect(this,&CobotMotomanTCPComm::resendCmd,this,&CobotMotomanTCPComm::onRensendCmd);

    connect(this, &CobotMotomanTCPComm::asyncServojFlushRequired,
            this, &CobotMotomanTCPComm::asyncServojFlush, Qt::QueuedConnection);
}

CobotMotomanTCPComm::~CobotMotomanTCPComm(){
    m_tcpSocket->close();
}

void CobotMotomanTCPComm::start(){
    if (m_host.isEmpty()) return;

    COBOT_LOG.info() << "Acquire firmware version: Connecting...";
    CobotMotomanFirmwareQueryer firmQueryer(m_host);
    if (firmQueryer.getVersion(m_robotState)) {
        /**
         * Link motoman general message channel
         */
        m_tcpSocket->connectToHost(m_host, TCP_PORT);//Motoman tcp port ID is 11000
    } else {
        Q_EMIT connectFail();
    }
}

void CobotMotomanTCPComm::setupHost(const QString& host){
    m_host = host;
}

void CobotMotomanTCPComm::stop(){
}

void CobotMotomanTCPComm::processData(){
    QByteArray msg = m_tcpSocket->readAll();
    if (msg.size() <=0) {
        //m_robotState->setDisconnected();
        m_tcpSocket->close();
        Q_EMIT resendCmd();
        return;
    }
    if(msg.size()!=2){
        COBOT_LOG.error()<<"The size of received Motoman TCP message is not 2 bytes.";
        Q_EMIT resendCmd();
        return;
    }
    if((quint8)msg[0]!=m_cmdID){
        COBOT_LOG.error()<<"Received Motoman TCP message ID error.";
        Q_EMIT resendCmd();
        return;
    }
    if((qint8)msg[1]!=0){
        Q_EMIT resendCmd();
        return;
    }
    if(m_LastCmdID==CMD_MOVE_ANGLE){
        asyncServojFlush();
    }
}

void CobotMotomanTCPComm::secConnectHandle(){
    COBOT_LOG.info() << "Secondary interface: Got connection";
    localIp_ = m_tcpSocket->localAddress().toString().toStdString();
    Q_EMIT connected();
}

std::string CobotMotomanTCPComm::getLocalIp(){
    return localIp_;
}

void CobotMotomanTCPComm::secDisconnectHandle(){
    COBOT_LOG.info() << "Secondary interface: disconnected";
    Q_EMIT disconnected();
}

void CobotMotomanTCPComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanComm: " << m_tcpSocket->errorString();
    Q_EMIT connectFail();
}
void CobotMotomanTCPComm::executeCmd(const ROBOTCMD CmdID,bool resendFlag) {
    std::vector<double> angleIncrement;
    angleIncrement.resize(6,0.0);
    ROBOTCMD Cmd2send;
    if(resendFlag){
        if(m_cmdID>0){
            m_cmdID--;
        }else {
            m_cmdID = 255;
        }
        Cmd2send=m_LastCmdID;
    }else{
        Cmd2send=CmdID;
    }
    m_LastCmdID=Cmd2send;
    QByteArray cmd;
    cmd.resize(5);
    QByteArray IP=IntToArray(m_tcpSocket->localAddress().toIPv4Address());
    switch (Cmd2send){
        case CMD_START_UDP:
            cmd[ 0] = 0xc0;
            cmd[ 1] = IP[0];
            cmd[ 2] = IP[1];
            cmd[ 3] = IP[2];
            cmd[ 4] = IP[3];
            break;
        case CMD_SERVO_ON:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0xff;
            cmd[ 3] = 0x00;
            break;
        case CMD_SERVO_OFF:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0x00;
            cmd[ 3] = 0x00;
            break;
        case CMD_MOVE_ANGLE:
            cmd[0]=0xc1;
            cmd[1]=0x0a;
            //cmd[2]=0x80;
            cmd[2]=0x81;
            for(int i=0;i<6;i++){
                //或许这里要做成比例式的。
                if(m_robotState.get()->getQActual()[i]-m_qTarget[i]>MAX_ANGLE_INCREMENT){
                    angleIncrement[i]=MAX_ANGLE_INCREMENT;
                }else if(m_robotState.get()->getQActual()[i]-m_qTarget[i]<-MAX_ANGLE_INCREMENT){
                    angleIncrement[i]=-MAX_ANGLE_INCREMENT;
                }else{
                    angleIncrement[i]=m_robotState.get()->getQActual()[i]-m_qTarget[i];
                }
                cmd.push_back(IntToArray(angleIncrement[i]));
                //Note No speed and accel control now.
            }
            break;
        case CMD_MOVE_IMPULSE:
            cmd[0]=0xc1;
            cmd[1]=0x0a;
            cmd[2]=0x80;
            for(int i=0;i<6;i++){
                //或许这里要做成比例式的。
                if(m_robotState.get()->getQActual()[i]-m_qTarget[i]>MAX_ANGLE_INCREMENT){
                    angleIncrement[i]=MAX_ANGLE_INCREMENT;
                }else if(m_robotState.get()->getQActual()[i]-m_qTarget[i]<-MAX_ANGLE_INCREMENT){
                    angleIncrement[i]=-MAX_ANGLE_INCREMENT;
                }else{
                    angleIncrement[i]=m_robotState.get()->getQActual()[i]-m_qTarget[i];
                }
                cmd.push_back(IntToArray(angleIncrement[i]));
                //Note No speed and accel control now.
            }
            break;
        default:
            COBOT_LOG.error()<<"Undefined command.";
    }
    //cmd.resize(CobotMotoman::FRAME_LENGTH_);
    cmd.resize(FRAME_LENGTH);
    sendCmd(cmd);
}

void CobotMotomanTCPComm::sendCmd(QByteArray &cmd) {
    if(cmd.size()!=FRAME_LENGTH){
        COBOT_LOG.error()<<"The size of tcp frame length is not "<<FRAME_LENGTH;
        return;
    }
    if(m_cmdID<255){
        m_cmdID++;
    }else {
        m_cmdID = 0;
    }
    cmd[FRAME_LENGTH-2]=m_cmdID;
    cmd[FRAME_LENGTH-1] = 0xff;
    if(m_tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        m_tcpSocket->write(cmd); //write the data itself
    }
    else{
        COBOT_LOG.error()<<"TCP socket state is QAbstractSocket::UnconnectedState.";
        return;
    }

}

void CobotMotomanTCPComm::onRensendCmd() {
    executeCmd(CMD_SERVO_OFF,true);
}
void CobotMotomanTCPComm::asyncServoj(const std::vector<double>& positions, bool flushNow){
    m_rt_res_mutex.lock();
    m_rt_q_required = positions;
    m_rt_res_mutex.unlock();

//    COBOT_LOG.info() << positions[0];

    if (flushNow) {
        Q_EMIT asyncServojFlushRequired();
    }
}

void CobotMotomanTCPComm::asyncServojFlush(){
    std::vector<double> tmpq;
    if (m_rt_res_mutex.try_lock()) {
        if (m_rt_q_required.size()) {
            m_qTarget = m_rt_q_required;
            m_rt_q_required.clear();
        }
        m_rt_res_mutex.unlock();
    }

    if (m_qTarget.size() == 0) {
        m_qTarget = m_robotState->getQActual();
    }
    //TODO How to start the loop need to design.
    executeCmd(CMD_MOVE_ANGLE);
}
void CobotMotomanTCPComm::stopProg(){
    if (keepalive) {
        keepalive = 0;
        COBOT_LOG.info() << "Stopping Motoman Driver Program";
        executeCmd(CMD_SERVO_OFF);
    }
}
