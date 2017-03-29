//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURREALTIMECOMM_H
#define PROJECT_COBOTURREALTIMECOMM_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTcpServer>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "../URDriver/robot_state_RT.h"

class CobotUrRealTimeComm : public QObject {
Q_OBJECT
public:
    CobotUrRealTimeComm(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent = nullptr);
    ~CobotUrRealTimeComm();

    void start();

    void readData();
    void onConnected();
    void onDisconnected();

    std::shared_ptr<RobotStateRT> getRobotState(){ return m_robotState; }

    void writeLine(const QByteArray& ba);

    void servoj(const std::vector<double>& j, int keepalive = 1);

Q_SIGNALS:
    void connected();
    void disconnected();
    void realTimeProgConnected();

protected:
    void urProgConnect();

protected:
    QString m_hostIp;
    std::shared_ptr<RobotStateRT> m_robotState;
    QTcpSocket* m_SOCKET;
    std::condition_variable& m_msg_cond;

    QTcpServer* m_tcpServer;
    QTcpSocket* m_rtSOCKET;

public:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
    const unsigned int REVERSE_PORT_ = 50007;

};


#endif //PROJECT_COBOTURREALTIMECOMM_H
