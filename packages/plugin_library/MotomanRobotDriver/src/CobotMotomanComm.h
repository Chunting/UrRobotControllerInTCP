//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_COMM_H
#define COBOT_MOTOMAN_COMM_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include "CobotMotomanUDPComm.h"
#include "CobotMotomanTCPComm.h"
class CobotMotomanComm : public QObject {
Q_OBJECT
public:
    CobotMotomanComm(std::condition_variable& rt_msg_cond,
                  std::condition_variable& msg_cond,
                  const QString& robotAddr,
                  QObject* parent = nullptr);
    ~CobotMotomanComm();


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
    CobotMotomanTCPCommCtrl* m_motomanTCPCommCtrl;
    CobotMotomanUDPCommCtrl* m_motomanUDPCommCtrl;

protected:
    void handleTCPConnected();
    void handleUDPConnected();
    void handleTCPDisconnected();
    void handleUDPDisconnected();

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


#endif //COBOT_MOTOMAN_COMM_H
