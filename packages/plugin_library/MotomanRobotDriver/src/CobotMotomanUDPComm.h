//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_UDP_COMM_H
#define COBOT_MOTOMAN_UDP_COMM_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QTcpServer>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "CobotMotoman.h"

class CobotMotomanUDPComm : public QObject {
Q_OBJECT
public:
    CobotMotomanUDPComm(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent = nullptr);
    ~CobotMotomanUDPComm();

    void start();
    void readData();
    void onConnected();

    std::shared_ptr<MotomanRobotState> getRobotState(){ return m_robotState; }


Q_SIGNALS:
    void connected();
    void disconnected();
    void connectFail();

protected:
    void onUDPDisconnect();
    void onSocketError(QAbstractSocket::SocketError socketError);


protected:
    QString m_robotIp;
    QString m_localIp;
    std::shared_ptr<MotomanRobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    QUdpSocket* m_udpSocket;
};

class CobotMotomanUDPCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotMotomanUDPComm* motoman;
    std::condition_variable& cond_msg;
public:
    CobotMotomanUDPCommCtrl(std::condition_variable& msg, const QString& robotIp, QObject* parent = nullptr)
            : QObject(parent), cond_msg(msg){
        motoman = new CobotMotomanUDPComm(msg, robotIp);
        motoman->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, motoman, &QObject::deleteLater);
        connect(this, &CobotMotomanUDPCommCtrl::start, motoman, &CobotMotomanUDPComm::start);
        connect(motoman, &CobotMotomanUDPComm::connected, this, &CobotMotomanUDPCommCtrl::onUDPConnected);
        workerThread.start();
    }

    ~CobotMotomanUDPCommCtrl(){
        workerThread.quit();
        workerThread.wait();
        COBOT_LOG.info() << "Cobot Motoman UDP Comm freed";
    }

    void startComm(){
        Q_EMIT start();
    }


Q_SIGNALS:
    void start();

protected:
    void onUDPConnected();
};

#endif //COBOT_MOTOMAN_UDP_COMM_H
