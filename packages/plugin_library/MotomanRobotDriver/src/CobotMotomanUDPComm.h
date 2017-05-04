//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_REALTIME_COMM_H
#define COBOT_MOTOMAN_REALTIME_COMM_H

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
    std::shared_ptr<MotomanRobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    QUdpSocket* m_udpSocket;
public:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
    const unsigned int REVERSE_PORT_ = 50007;
    int keepalive;
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
        COBOT_LOG.info() << "CobotUrRealTimeCommCtrl freed";
    }

    void startComm(){
        Q_EMIT start();
    }

    void addCommandToQueue(const QByteArray& ba){
        Q_EMIT commandReady(ba);
    }

    void requireStopServoj(){
        Q_EMIT stopServoj();
    }

Q_SIGNALS:
    void start();
    void commandReady(const QByteArray& ba);
    void stopServoj();

protected:
    void onUDPConnected();

private:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
};

#endif //COBOT_MOTOMAN_REALTIME_COMM_H
