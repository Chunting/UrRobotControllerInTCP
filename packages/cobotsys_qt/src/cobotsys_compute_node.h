//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_COMPUTE_NODE_H
#define PROJECT_COBOTSYS_COMPUTE_NODE_H

#include <QObject>
#include <QHostAddress>
#include <QTcpServer>
#include <QTcpSocket>
#include <QString>
#include <QTimer>
#include <cobotsys_compute_config.h>
#include <cobotsys_qt.h>

namespace cobotsys {


class ComputeNode : public QObject {
Q_OBJECT
public:

public:
    ComputeNode(QObject *parent = nullptr);
    ~ComputeNode();

    void connectMaster(const server::CONFIG &config = server::CONFIG());
    void writeData(const QByteArray &ba);

Q_SIGNALS:
    void masterConnected();
    void masterDisconnected();

protected:
    void onMasterConnect();
    void onMasterDisconnect();
    void onMasterFound();
    void onError(QAbstractSocket::SocketError error);

    void onDataReady();
protected:
    virtual void processData(const QByteArray &ba);
    virtual void connectHost(int delayMSec = 0);

    virtual void processConnect();
    virtual void processDisconnect();
protected:
    QTcpSocket *_client;
    server::CONFIG _config;
    bool _is_connected;
    int _re_connect_delay;
};



//
}

#endif //PROJECT_COBOTSYS_COMPUTE_NODE_H
