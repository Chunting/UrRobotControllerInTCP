//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_COMM_CTRL_H
#define COBOT_MOTOMAN_COMM_CTRL_H

#include <QObject>
#include <QThread>
#include <cobotsys_logger.h>
#include "CobotMotomanComm.h"
//TODO DONE
class CobotMotomanCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotMotomanComm* motoman;

public:
    CobotMotomanCommCtrl(std::condition_variable& cond_msg, const QString& hostIp, QObject* parent = nullptr)
            : QObject(parent){
        motoman = new CobotMotomanComm(cond_msg);
        motoman->setupHost(hostIp);
        motoman->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, motoman, &QObject::deleteLater);
        connect(this, &CobotMotomanCommCtrl::start, motoman, &CobotMotomanComm::start);
        workerThread.start();
    }

    ~CobotMotomanCommCtrl(){
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


#endif //COBOT_MOTOMAN_COMM_CTRL_H
