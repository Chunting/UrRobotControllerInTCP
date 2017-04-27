//
// Created by 杨帆 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
//#include <cobotsys_logger.h>
//#include <QtCore/QJsonObject>
//#include <QtCore/qjsonarray.h>
#include <extra2.h>
#include "MotomanRobotDriver.h"


MotomanRobotDriver::MotomanRobotDriver(QObject *parent) :
        QObject(parent),
        m_isOnline(false),
        m_cmdID(0),
        m_address(""),
        m_tcp_port(11000),
        m_udp_port(11001){
    m_translation.x() = 0.0;
    m_translation.y() = 0.0;
    m_translation.z() = 0.0;

    m_currentPosition.resize(6,0.0);
    m_targetPosition.resize(6,0.0);
    m_speed.resize(6,0.0);
    m_accel.resize(6,0.0);

    m_tcpSocket = std::make_shared<QTcpSocket>();
    m_udpSocket =std::make_shared<QUdpSocket>();
    qRegisterMetaType<QAbstractSocket::SocketError>("SocketError");
    QObject::connect(m_tcpSocket.get(), SIGNAL(connected()), this, SLOT(init()), Qt::DirectConnection);
    QObject::connect(m_tcpSocket.get(), SIGNAL(disconnected()), this, SLOT(offline()), Qt::DirectConnection);
    QObject::connect(m_tcpSocket.get(), SIGNAL(readyRead()), this, SLOT(receivedMessage()), Qt::DirectConnection);
    QObject::connect(m_tcpSocket.get(), SIGNAL(error(QAbstractSocket::SocketError)),
                     this, SLOT(receivedError(QAbstractSocket::SocketError)), Qt::DirectConnection);
    QObject::connect(m_udpSocket.get(),SIGNAL(readyRead()),this,SLOT(readPendingDatagrams()), Qt::DirectConnection);
}

void MotomanRobotDriver::move(const std::vector<double> &q) {
    if(q.size()!=6){
        COBOT_LOG.error()<<"The size of Motoman robot joint position is not 6";
        return;
    }
    m_targetPosition=q;
}
void MotomanRobotDriver::MoveTread(){
    //0.0001度。
    QByteArray motionCmd;
    //TODO 将这个做成一个单独的线程。
    std::vector<double> angleIncrement;
    angleIncrement.resize(6,0.0);
    //m_speed  m_accel
    //这里面要做运动规划。位置增量不能超过1度。
    //0xc1      0x0a        0x80    0x00 0x00 +24字节位置信息+24字节速度信息+24字节加速度信息+cmdID+0xff
    //move指令   增量模式     脉冲模式
    //0xc1      0x0a        0x81    0x00 0x00 +24字节位置信息+24字节速度信息+24字节加速度信息+cmdID+0xff
    //move指令   增量模式     角度模式
    motionCmd.resize(5);//首先将vector长度设置为5
    motionCmd[0]=0xc1;
    motionCmd[1]=0x0a;
    //motionCmd[2]=0x80;
    motionCmd[2]=0x81;

    while(!m_stop){
        motionCmd.resize(5);
        for(int i=0;i<6;i++){
            //或许这里要做成比例式的。
            if(m_targetPosition[i]-m_currentPosition[i]>MAX_ANGLE_INCREMENT){
                angleIncrement[i]=MAX_ANGLE_INCREMENT;
            }else if(m_targetPosition[i]-m_currentPosition[i]<-MAX_ANGLE_INCREMENT){
                angleIncrement[i]=-MAX_ANGLE_INCREMENT;
            }else{
                angleIncrement[i]=m_targetPosition[i]-m_currentPosition[i];
            }
            motionCmd.push_back(IntToArray(angleIncrement[i]));
            //TODO No speed and accel control now.
        }
        motionCmd.resize(FRAME_LENGTH);
    }
}
void MotomanRobotDriver::readPendingDatagrams(){
    //TODO parse information needed.
    COBOT_LOG.notice() << "readPendingDatagrams";
}
std::shared_ptr<AbstractDigitIoDriver> MotomanRobotDriver::getDigitIoDriver(int deviceId) {
    if (deviceId == 0)
        return m_digitOutput;
    if (deviceId == 1)
        return m_digitInput;
    return nullptr;
}

bool setup(const QString &configFilePath) {
    return true;
}
void MotomanRobotDriver::offline()
{
    COBOT_LOG.notice() << "Robot Motoman is offline.";
    m_isOnline = false;
}

void MotomanRobotDriver::init()
{
    m_isOnline = true;
}

bool MotomanRobotDriver::start() {
    m_isOnline = true;
    COBOT_LOG.notice() << "Robot IP Address: " << m_address;
    COBOT_LOG.notice() << "Robot TCP Port: " << m_tcp_port;
    COBOT_LOG.notice() << "Robot UDP Port: " << m_udp_port;

    m_tcpSocket->abort();
    m_tcpSocket->connectToHost(m_address, m_tcp_port);

    QHostAddress *robotAddress  = new QHostAddress(m_address);
    m_udpSocket->abort();
    //bind and connect to host 哪个在前，哪个在后，有待确定。
    m_udpSocket->bind(*robotAddress, m_udp_port);
    m_udpSocket->connectToHost(m_address,m_udp_port);

    if (!m_tcpSocket->waitForConnected()) {
        COBOT_LOG.error() << "Failed to connect the tcp port of Motoman robot. IP:"<<m_address<<" tcp port:"<<m_tcp_port;
        return false;
    }

    if (!m_udpSocket->waitForConnected()) {
        COBOT_LOG.error() << "Failed to connect the udp port of Motoman Robot. IP:"<<m_address<<" udp port:"<<m_udp_port;
        return false;
    }
    return true;
}

void MotomanRobotDriver::stop() {
    m_tcpSocket->disconnectFromHost();
    m_tcpSocket->close();
    m_udpSocket->disconnectFromHost();
    m_udpSocket->close();
}

QString MotomanRobotDriver::getRobotUrl() {
    return QString();
}

void MotomanRobotDriver::attach(const std::shared_ptr<ArmRobotRealTimeStatusObserver> &observer) {

}

Eigen::Vector3f MotomanRobotDriver::position() {
    return m_translation;
}

QByteArray MotomanRobotDriver::IntToArray(qint32 source) //Use qint32 to ensure that the number have 4 bytes
{
    //Avoid use of cast, this is the Qt way to serialize objects
    QByteArray temp;
    QDataStream data(&temp, QIODevice::ReadWrite);
    data << source;
    return temp;
}

bool MotomanRobotDriver::executeCmd(const ROBOTCMD CmdID){
    QByteArray cmd;
    cmd.resize(FRAME_LENGTH);
    QHostAddress *robotAddress  = new QHostAddress(m_address);
    QByteArray IP=IntToArray(robotAddress->toIPv4Address());
    switch (CmdID){
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
        default:
            COBOT_LOG.error()<<"Undefined command.";
    }
    return sendCmd(cmd);
}
bool MotomanRobotDriver::sendCmd(QByteArray &cmd) {
    if(cmd.size()!=FRAME_LENGTH){
        COBOT_LOG.error()<<"The size of tcp frame length is not "<<FRAME_LENGTH;
        return false;
    }
    if(m_cmdID<255){
        m_cmdID++;
    }else {
        m_cmdID = 0;
    }
    cmd[FRAME_LENGTH-2]=m_cmdID;
    cmd[FRAME_LENGTH-1] = 0xff;
    m_executeSuccess = false;
    if(m_tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        //m_tcpSocket->write(IntToArray(cmd.size())); //write size of data
        m_tcpSocket->write(cmd); //write the data itself
        return m_tcpSocket->waitForBytesWritten();
    }
    else
        return false;
}

void MotomanRobotDriver::receivedMessage() {
    QByteArray msg = m_tcpSocket->readAll();
    if(msg.size()!=2){
        COBOT_LOG.error()<<"The size of received Motoman TCP message is not 2 bytes.";
        m_executeSuccess=false;
        return;
    }
    if((quint8)msg[0]!=m_cmdID){
        COBOT_LOG.error()<<"Received Motoman TCP message ID error.";
        m_executeSuccess=false;
        return;
    }
    if((qint8)msg[1]!=0){
        m_executeSuccess=false;
        return;
    }
    m_executeSuccess=true;
}

bool MotomanRobotDriver::setup(const QString &configFilePath) {
    //auto a = FileFinder::find(configFilePath.toStdString);
    //load model from json file.

    COBOT_LOG.info() << "configFilePath" << configFilePath.toStdString();
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_address=json["robot_ip"].toString();
        return true;
    }else{
        return false;
    }
}

void MotomanRobotDriver::receivedError(QAbstractSocket::SocketError socketError) {
    switch (socketError) {
        case QAbstractSocket::RemoteHostClosedError:
            break;
        case QAbstractSocket::HostNotFoundError:
            COBOT_LOG.error() << "The host was not found. Please check the host name and port settings.";
            break;
        case QAbstractSocket::ConnectionRefusedError:
            COBOT_LOG.error()  << "The connection was refused by the peer. Make sure the fortune server is running,"
                    " and check that the host name and port settings are correct.";
            break;
        default:
            COBOT_LOG.error()  << QString("The following error occurred: %1.").arg(m_tcpSocket->errorString());
    }
}