//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURCOMMUNICATIONCONTROLLER_H
#define PROJECT_COBOTURCOMMUNICATIONCONTROLLER_H

#include <QObject>
#include <QThread>
#include <cobotsys_logger.h>
#include <extra2.h>
#include "CobotUrComm.h"

class CobotUrCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotUrComm* ur;
    std::condition_variable cond_msg;
public:
    CobotUrCommCtrl(const QString& hostIp, QObject* parent = nullptr)
            : QObject(parent){
        ur = new CobotUrComm(cond_msg);
        ur->setupHost(hostIp);
        ur->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, ur, &QObject::deleteLater);
        connect(this, &CobotUrCommCtrl::start, ur, &CobotUrComm::start);
        workerThread.start();
    }

    ~CobotUrCommCtrl(){
        workerThread.quit();
        workerThread.wait();
        INFO_DESTRUCTOR(this);
    }

    void startComm(){
        Q_EMIT start();
    }

Q_SIGNALS:
    void start();
};


#endif //PROJECT_COBOTURCOMMUNICATIONCONTROLLER_H
