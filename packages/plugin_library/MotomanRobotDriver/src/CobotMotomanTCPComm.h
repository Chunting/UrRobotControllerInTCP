//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_COMM_H
#define COBOT_MOTOMAN_COMM_H

#include <QObject>
#include <QString>
#include <QThread>
#include <cobotsys_logger.h>
#include <QTcpSocket>
#include <memory>
#include <thread>
#include <QSemaphore>
#include "CobotMotoman.h"

class CobotMotomanTCPComm : public QObject {
Q_OBJECT
public:
    CobotMotomanTCPComm(std::condition_variable& cond_msg, QObject* parent = nullptr);
    ~CobotMotomanTCPComm();

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
    void sendCmd(QByteArray cmd);
    void asyncServojFlushRequired();

public:

    enum ROBOTCMD {
        CMD_START_UDP,      //启动UDP通信
        CMD_SERVO_ON,       //开启伺服
        CMD_SERVO_OFF,      //关闭伺服
        CMD_QUERY_VERSION,  //查询固件版本
        CMD_SET_DO,         //设置DO
        CMD_MOVE_ANGLE,     //让机器人按照角度指令运动
        CMD_MOVE_IMPULSE    //让机器人按照脉冲指令运动
    };

    void start();
    void stop();
    void setDigitOut(int portIndex, bool b);
    std::string getVersion();
    void executeCmd(const ROBOTCMD CmdID,bool resendFlag=false);
protected:

    void processData();
    void connectHandle();
    void disconnectHandle();
    void onSocketError(QAbstractSocket::SocketError socketError);
    void asyncServojFlush();


protected Q_SLOTS:
    void onRensendCmd();
    void onSendCmd(QByteArray cmd);

protected:
    QTcpSocket* m_tcpSocket;
    QString m_host;
    quint8 m_lastMotionCmdIndex;
    std::shared_ptr<MotomanRobotState> m_robotState;
    std::condition_variable& m_msg_cond;
    std::string localIp_;
    std::vector<QByteArray> m_sentCmdCache;
    std::mutex m_rt_res_mutex;
    std::vector<double> m_rt_q_required;
    std::vector<double> m_qTarget;
    quint8 m_do_id;//被设置的数字量输出ID号，暂定为16个。
    bool m_do_bool_value;//被设置的数字量输出值。
    int keepalive;
};


class CobotMotomanTCPCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotMotomanTCPComm* motoman;

public:
    CobotMotomanTCPCommCtrl(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent = nullptr)
            : QObject(parent){
        motoman = new CobotMotomanTCPComm(cond_msg);
        motoman->setupHost(hostIp);
        motoman->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, motoman, &QObject::deleteLater);
        connect(this, &CobotMotomanTCPCommCtrl::start, motoman, &CobotMotomanTCPComm::start);
        workerThread.start();
    }

    ~CobotMotomanTCPCommCtrl(){
        workerThread.quit();
        workerThread.wait();
        COBOT_LOG.info() << "CobotMotomanCommCtrl freed";
    }

    void startComm(){
        Q_EMIT start();
    }

Q_SIGNALS:
    void start();
};


#endif //COBOT_MOTOMAN_COMM_H
