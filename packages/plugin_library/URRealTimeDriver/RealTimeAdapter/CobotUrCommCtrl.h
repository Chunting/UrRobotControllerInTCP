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
#include "CobotUr.h"

class CobotUrCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotUrComm* ur;
    std::condition_variable cond_msg;
    std::shared_ptr<ref_num> ref_num_;
public:
    CobotUrCommCtrl(std::shared_ptr<ref_num>& refNum, const QString& hostIp, QObject* parent = nullptr)
            : QObject(parent) {
        ref_num_ = refNum;
        ref_num_->add_ref();
        ur = new CobotUrComm(cond_msg);
        ur->setupHost(hostIp);
        ur->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, ur, &QObject::deleteLater);
        connect(this, &CobotUrCommCtrl::start, ur, &CobotUrComm::start);
        workerThread.start();
    }

    ~CobotUrCommCtrl() {
        workerThread.quit();
        workerThread.wait();
        INFO_DESTRUCTOR(this);
        ref_num_->dec_ref();
    }

    void startComm() {
        Q_EMIT start();
    }

Q_SIGNALS:
    void start();
};


#endif //PROJECT_COBOTURCOMMUNICATIONCONTROLLER_H
