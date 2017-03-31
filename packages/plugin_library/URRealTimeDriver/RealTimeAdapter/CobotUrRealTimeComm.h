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

    void servoj(const std::vector<double>& j);

    void stopProg();

    /**
     * 这个函数是专门写来用于异步线程发送命令的，可以直接调用
     * @param positions
     * @param flushNow
     */
    void asyncServoj(const std::vector<double>& positions, bool flushNow = false);


Q_SIGNALS:
    void connected();
    void disconnected();
    void connectFail();

    void realTimeProgConnected();
    void realTimeProgDisconnect();

    void asyncServojFlushRequired();

protected:
    void urProgConnect();
    void onRealTimeDisconnect();
    void asyncServojFlush();
    void onSocketError(QAbstractSocket::SocketError socketError);
protected:
    QString m_hostIp;
    std::shared_ptr<RobotStateRT> m_robotState;
    QTcpSocket* m_SOCKET;
    std::condition_variable& m_msg_cond;

    QTcpServer* m_tcpServer;
    QTcpSocket* m_rtSOCKET;

    std::mutex m_rt_res_mutex;
    std::vector<double> m_rt_q_required;

    std::vector<double> m_qTarget;

public:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
    const unsigned int REVERSE_PORT_ = 50007;
    int keepalive;
};


#endif //PROJECT_COBOTURREALTIMECOMM_H
