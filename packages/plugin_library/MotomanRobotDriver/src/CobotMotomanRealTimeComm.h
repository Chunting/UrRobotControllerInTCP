//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_REALTIME_COMM_H
#define COBOT_MOTOMAN_REALTIME_COMM_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTcpServer>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "motoman_robot_state.h"
#include "CobotMotoman.h"
//TODO DONE
class CobotMotomanRealTimeComm : public QObject {
Q_OBJECT
public:
    CobotMotomanRealTimeComm(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent = nullptr);
    ~CobotMotomanRealTimeComm();

    void start();

    void readData();
    void onConnected();
    void onDisconnected();

    std::shared_ptr<RobotState> getRobotState(){ return m_robotState; }

    /**
     * 发送给UR的即时脚本命令。
     * @param ba
     */
    void writeLine(const QByteArray& ba);

    void servoj(const std::vector<double>& j);

    void stopProg();




Q_SIGNALS:
    void connected();
    void disconnected();
    void connectFail();

    void realTimeProgConnected();
    void realTimeProgDisconnect();

protected:
    void motomanProgConnect();
    void onRealTimeDisconnect();
    void onSocketError(QAbstractSocket::SocketError socketError);


protected:
    QString m_robotIp;
    std::shared_ptr<RobotState> m_robotState;
    QTcpSocket* m_SOCKET;
    std::condition_variable& m_msg_cond;

    QTcpServer* m_tcpServer;
    QTcpSocket* m_rtSOCKET;

public:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
    const unsigned int REVERSE_PORT_ = 50007;
    int keepalive;
};


#endif //COBOT_MOTOMAN_REALTIME_COMM_H
