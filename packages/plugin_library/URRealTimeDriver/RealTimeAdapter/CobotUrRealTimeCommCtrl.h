//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURREALTIMECOMMCTRL_H
#define PROJECT_COBOTURREALTIMECOMMCTRL_H

#include <QObject>
#include <QThread>
#include <cobotsys_logger.h>
#include <extra2.h>
#include "CobotUrRealTimeComm.h"
#include "CobotUr.h"

class CobotUrRealTimeCommCtrl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    CobotUrRealTimeComm* ur;
    std::shared_ptr<std::condition_variable> cond_msg;
    std::shared_ptr<ref_num> ref_num_;
public:
    CobotUrRealTimeCommCtrl(std::shared_ptr<ref_num>& refNum,
                            std::shared_ptr<std::condition_variable>& msg,
                            const QString& hostIp, QObject* parent = nullptr)
            : QObject(parent) {
        ref_num_ = refNum;
        ref_num_->add_ref();
        cond_msg = msg;
        ur = new CobotUrRealTimeComm(*cond_msg.get(), hostIp);
        ur->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, ur, &QObject::deleteLater);
        connect(this, &CobotUrRealTimeCommCtrl::start, ur, &CobotUrRealTimeComm::start);
        connect(this, &CobotUrRealTimeCommCtrl::commandReady, ur, &CobotUrRealTimeComm::writeLine);
        connect(ur, &CobotUrRealTimeComm::connected, this, &CobotUrRealTimeCommCtrl::onRealTimeConnected);
        connect(this, &CobotUrRealTimeCommCtrl::stopServoj, ur, &CobotUrRealTimeComm::stopProg);
        workerThread.start();
    }

    ~CobotUrRealTimeCommCtrl() {
        workerThread.quit();
        workerThread.wait();
        INFO_DESTRUCTOR(this);
        ref_num_->dec_ref();
    }

    void startComm() {
        Q_EMIT start();
    }

    void addCommandToQueue(const QByteArray& ba) {
        Q_EMIT commandReady(ba);
    }

    void requireStopServoj() {
        Q_EMIT stopServoj();
    }

Q_SIGNALS:
    void start();
    void commandReady(const QByteArray& ba);
    void stopServoj();

protected:
    void onRealTimeConnected();

private:
    const int MULT_JOINTSTATE_ = 1000000;
    const int MULT_TIME_ = 1000000;
};

#endif //PROJECT_COBOTURREALTIMECOMMCTRL_H
