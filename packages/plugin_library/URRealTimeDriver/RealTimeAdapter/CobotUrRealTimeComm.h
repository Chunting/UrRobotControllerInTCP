//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURREALTIMECOMM_H
#define PROJECT_COBOTURREALTIMECOMM_H

#include <QObject>
#include <QString>
#include <QTcpSocket>
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

Q_SIGNALS:
    void connected();
    void disconnected();
protected:
    QString m_hostIp;
    std::shared_ptr<RobotStateRT> m_robotState;
    QTcpSocket* m_SOCKET;
    std::condition_variable& m_msg_cond;
};


#endif //PROJECT_COBOTURREALTIMECOMM_H
