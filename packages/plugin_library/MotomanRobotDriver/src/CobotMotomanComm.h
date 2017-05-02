//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_COMM_H
#define COBOT_MOTOMAN_COMM_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "CobotMotoman.h"
#include "motoman_robot_state.h"

//TODO DONE
class CobotMotomanComm : public QObject {
Q_OBJECT
public:
    CobotMotomanComm(std::condition_variable& cond_msg, QObject* parent = nullptr);
    ~CobotMotomanComm();

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

    void sendCmd(QByteArray& cmd);
    void executeCmd(const CobotMotoman::ROBOTCMD CmdID);

protected:
    void processData();
    void secConnectHandle();
    void secDisconnectHandle();
    void onSocketError(QAbstractSocket::SocketError socketError);

protected:
    QTcpSocket* m_tcpSocket;
    QString m_host;
    std::shared_ptr<RobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    std::string localIp_;
    quint8 m_cmdID;//motoman cmd ID
};


#endif //COBOT_MOTOMAN_COMM_H
