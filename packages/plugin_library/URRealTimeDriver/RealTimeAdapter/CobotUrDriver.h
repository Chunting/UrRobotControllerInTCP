//
// Created by 潘绪洋 on 17-3-29.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURDRIVER_H
#define PROJECT_COBOTURDRIVER_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include "CobotUrRealTimeCommCtrl.h"
#include "CobotUrCommCtrl.h"

class CobotUrDriver : public QObject {
Q_OBJECT
public:
    CobotUrDriver(std::condition_variable& rt_msg_cond,
                  std::condition_variable& msg_cond,
                  const QString& robotAddr,
                  QObject* parent = nullptr);
    ~CobotUrDriver();


    void startDriver();
    void stopDriver();

    void setServojTime(double t);
    void setServojLookahead(double t);
    void setServojGain(double g);

    void servoj(const std::vector<double>& positions);

Q_SIGNALS:
    void driverStartFailed();
    void driverStartSuccess();
    void driverStopped();


public:
    CobotUrCommCtrl* m_urCommCtrl;
    CobotUrRealTimeCommCtrl* m_urRealTimeCommCtrl;

protected:
    void handleCommConnected();
    void handleRTCommConnected();
    void handleDisconnected();
    void handleRTProgConnect();
    void handleRTProgDisconnect();

    bool uploadProg();
    void onConnectSuccess();
protected:
    bool m_noDisconnectedAccept;

private:

    double servoj_time_;
    double servoj_lookahead_time_;
    double servoj_gain_;
    std::string ip_addr_;

    int m_disconnectCount;

    int m_connectTime;
    bool m_isConnected;
};


#endif //PROJECT_COBOTURDRIVER_H
