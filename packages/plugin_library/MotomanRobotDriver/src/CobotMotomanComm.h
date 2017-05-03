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

class CobotMotomanComm : public QObject {
Q_OBJECT
public:
    CobotMotomanComm(std::condition_variable& cond_msg, QObject* parent = nullptr);
    ~CobotMotomanComm();

    void setupHost(const QString& host);

    std::shared_ptr<MotomanRobotState> getRobotState(){ return m_robotState; }
    std::string getLocalIp();

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
    void resendCmd();

    void asyncServojFlushRequired();
public:
    void start();
    void stop();

    void sendCmd(QByteArray& cmd);
    void executeCmd(const CobotMotoman::ROBOTCMD CmdID,bool resendFlag=false);
    void stopProg();
protected:
    void processData();
    void secConnectHandle();
    void secDisconnectHandle();
    void onSocketError(QAbstractSocket::SocketError socketError);
    void asyncServojFlush();
protected Q_SLOTS:
    void onRensendCmd();
protected:
    QTcpSocket* m_tcpSocket;
    QString m_host;
    std::shared_ptr<MotomanRobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    std::string localIp_;
    quint8 m_cmdID;//motoman cmd ID

    CobotMotoman::ROBOTCMD m_LastCmdID;
    std::mutex m_rt_res_mutex;
    std::vector<double> m_rt_q_required;
    std::vector<double> m_qTarget;

    int keepalive;
};


#endif //COBOT_MOTOMAN_COMM_H
