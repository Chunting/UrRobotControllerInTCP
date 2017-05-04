//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtNetwork/QHostAddress>
#include "CobotMotomanTCPComm.h"

CobotMotomanTCPComm::CobotMotomanTCPComm(std::condition_variable& cond_msg, QObject* parent)
        : QObject(parent), m_msg_cond(cond_msg){
    m_lastMotionCmdIndex=1;
    m_sentCmdCache.clear();
    m_robotState = std::make_shared<MotomanRobotState>(m_msg_cond);

    m_tcpSocket = new QTcpSocket(this);

    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &CobotMotomanTCPComm::processData);
    connect(m_tcpSocket, &QTcpSocket::connected, this, &CobotMotomanTCPComm::connectHandle);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &CobotMotomanTCPComm::disconnectHandle);
    connect(m_tcpSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &CobotMotomanTCPComm::onSocketError);

    connect(this,static_cast<void (CobotMotomanTCPComm::*)(QByteArray)>(&CobotMotomanTCPComm::sendCmd),this,&CobotMotomanTCPComm::onSendCmd);
    connect(this,&CobotMotomanTCPComm::resendCmd,this,&CobotMotomanTCPComm::onRensendCmd);

    connect(this, &CobotMotomanTCPComm::asyncServojFlushRequired,
            this, &CobotMotomanTCPComm::asyncServojFlush, Qt::QueuedConnection);
}

CobotMotomanTCPComm::~CobotMotomanTCPComm(){
    m_tcpSocket->close();
}

void CobotMotomanTCPComm::start(){
    if (!m_host.isEmpty()){
        COBOT_LOG.notice()<<"Robot IP:"<<m_host<<" TCP Port:"<<TCP_PORT;
        m_tcpSocket->connectToHost(m_host, TCP_PORT);//Motoman tcp port ID is 11000
    }
}

void CobotMotomanTCPComm::setupHost(const QString& host){
    m_host = host;
}

void CobotMotomanTCPComm::stop(){
    if (keepalive) {
        keepalive = 0;
        COBOT_LOG.info() << "Stopping Motoman Driver";
        executeCmd(CMD_SERVO_OFF);
    }
}

void CobotMotomanTCPComm::connectHandle(){
    COBOT_LOG.info() << "Motoman TCP sockect connected";
    localIp_ = m_tcpSocket->localAddress().toString().toStdString();
    Q_EMIT connected();
}

std::string CobotMotomanTCPComm::getLocalIp(){
    return localIp_;
}

void CobotMotomanTCPComm::disconnectHandle(){
    COBOT_LOG.info() << "Motoman TCP sockect disconnected";
    Q_EMIT disconnected();
}

void CobotMotomanTCPComm::onSocketError(QAbstractSocket::SocketError socketError){
    COBOT_LOG.error() << "CobotMotomanComm: " << m_tcpSocket->errorString();
    Q_EMIT connectFail();
}

void CobotMotomanTCPComm::processData(){
    QByteArray msg = m_tcpSocket->readAll();
    QByteArray checkedCmd;
    if(m_sentCmdCache.size()>0){
        checkedCmd=m_sentCmdCache[0];
        m_sentCmdCache.erase(m_sentCmdCache.begin());
    }
    COBOT_LOG.debug()<<"The Command received:"<<QString(msg.toHex());
    if (msg.size() <=0) {
        //m_robotState->setDisconnected();
        m_tcpSocket->close();
        return;
    }
    //TODO 假定TCP/IP通信很理想。帧长能正确返回。为了通信准确，未来需要改进。
//    if(msg.size()!=2){
//        COBOT_LOG.error()<<"The size of received Motoman TCP message is not 2 bytes.";
//        Q_EMIT resendCmd();
//        return;
//    }
    //TODO 假定TCP/IP通信很理想。ID能正确返回。为了通信准确，未来需要改进。
//    if((quint8)msg[0]!=checkedCmd[FRAME_LENGTH-2]){
//        COBOT_LOG.error()<<"Received Motoman TCP message ID error.";
//        Q_EMIT resendCmd();
//        return;
//    }
    //TODO 暂时不处理错误码信息。
//    if((qint8)msg[1]!=0){
//        COBOT_LOG.error()<<"Motoman Controller execute error, resend cmd.";
//        Q_EMIT resendCmd();
//        return;
//    }
    if((quint8)msg[0]==m_lastMotionCmdIndex){
        asyncServojFlush();
    }
    //TODO: it is dangerous to debug now!
}

void CobotMotomanTCPComm::executeCmd(const ROBOTCMD CmdID,bool resendFlag) {
    static quint8 cmdIndex=0;//motoman cmd ID
    std::vector<double> angleIncrement;
    angleIncrement.resize(6,0.0);
    QByteArray cmd;
    cmd.resize(5);
    QByteArray IP=IntToArray(m_tcpSocket->localAddress().toIPv4Address());
    switch (CmdID){
        case CMD_START_UDP:
            cmd[ 0] = 0xc0;
            cmd[ 1] = IP[0];
            cmd[ 2] = IP[1];
            cmd[ 3] = IP[2];
            cmd[ 4] = IP[3];
            COBOT_LOG.debug()<<"CMD_START_UDP is sent";
            break;
        case CMD_SERVO_ON:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0xff;
            cmd[ 3] = 0x00;
            COBOT_LOG.debug()<<"CMD_SERVO_ON is sent";
            break;
        case CMD_SERVO_OFF:
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0xff;
            cmd[ 2] = 0x00;
            cmd[ 3] = 0x00;
            COBOT_LOG.debug()<<"CMD_SERVO_OFF is sent";
            break;
        case CMD_QUERY_VERSION:
            cmd[ 0] = 0xc3;
            COBOT_LOG.debug()<<"CMD_QUERY_VERSION is sent";
            break;
        case CMD_SET_DO://
            cmd[ 0] = 0xc2;
            cmd[ 1] = 0x00;
            //cmd[ 2] = 0x00;
            cmd[ 3] = 0xff;
            cmd[4] = m_do_id;
            cmd[5] = (quint8)m_do_bool_value;//false设置为0，true设置为非0。
            COBOT_LOG.debug()<<"CMD_SET_DO is sent";
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
                cmd.push_back(IntToArray((qint32)(angleIncrement[i]*FLOAT_PRECISION)));
                //Note No speed and accel control now.
            }
            COBOT_LOG.debug()<<"CMD_MOVE_ANGLE is sent";
            break;
        case CMD_MOVE_IMPULSE:
            COBOT_LOG.debug()<<"CMD_MOVE_IMPULSE is sent";
            COBOT_LOG.warning()<<"Not develop yet.";
            return;
            break;
        default:
            COBOT_LOG.error()<<"Undefined command.";
            return;
    }
    cmd.resize(FRAME_LENGTH);
    if(cmdIndex<255){
        cmdIndex++;
    }else {
        cmdIndex = 0;
    }
    cmd[FRAME_LENGTH-2] = cmdIndex;
    cmd[FRAME_LENGTH-1] = 0xff;
    if(CmdID==CMD_MOVE_ANGLE){
        m_lastMotionCmdIndex=cmdIndex;
    }
    Q_EMIT sendCmd(cmd);
}

void CobotMotomanTCPComm::onSendCmd(QByteArray cmd) {

    if(cmd.size()!=FRAME_LENGTH){
        COBOT_LOG.error()<<"The size of tcp frame length is not "<<FRAME_LENGTH;
        return;
    }

    COBOT_LOG.debug()<<"The Command to be sent:"<<QString(cmd.toHex());
    if(m_tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        m_tcpSocket->write(cmd); //write the data itself
        m_sentCmdCache.push_back(cmd);
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
    executeCmd(CMD_MOVE_ANGLE);
}

void CobotMotomanTCPComm::setDigitOut(int portIndex, bool b) {
    m_do_id=(quint8)portIndex;
    m_do_bool_value=b;
    executeCmd(CMD_SET_DO);
}

std::string CobotMotomanTCPComm::getVersion() {
    COBOT_LOG.warning() << "Not develop yet.";
    return "1.0";
}
