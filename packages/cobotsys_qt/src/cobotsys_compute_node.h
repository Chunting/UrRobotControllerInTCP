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
    QTcpSocket *_client;
    server::CONFIG _config;
};



//
}

#endif //PROJECT_COBOTSYS_COMPUTE_NODE_H
