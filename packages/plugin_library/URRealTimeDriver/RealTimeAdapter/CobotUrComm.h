//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URCOMMUNICATION_H
#define PROJECT_URCOMMUNICATION_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "../URDriver/robot_state.h"

class CobotUrComm : public QObject {
Q_OBJECT
public:
    CobotUrComm(std::condition_variable& cond_msg, QObject* parent = nullptr);
    ~CobotUrComm();

    void setupHost(const QString& host);

    std::shared_ptr<RobotState> getRobotState(){ return m_robotState; }
    std::string getLocalIp();
Q_SIGNALS:
    void connected();
    void disconnected();
    void connectFail();

public:
    void start();
    void stop();

protected:
    void secReadData();
    void secConnectHandle();
    void secDisconnectHandle();
    void onSocketError(QAbstractSocket::SocketError socketError);

protected:
    QTcpSocket* m_tcpSocket;
    QString m_host;
    std::shared_ptr<RobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    std::string localIp_;
};


#endif //PROJECT_URCOMMUNICATION_H
