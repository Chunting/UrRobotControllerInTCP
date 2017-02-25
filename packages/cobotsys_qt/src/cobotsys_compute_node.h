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

namespace cobotsys {


class ComputeNode : public QObject {
Q_OBJECT
public:
    struct MasterConfig {
        QHostAddress address;
        uint16_t port;
    };

public:
    ComputeNode(QObject *parent = nullptr);
    ~ComputeNode();


    void connectMaster(const MasterConfig &config);

Q_SIGNALS:
    void masterConnected();


protected:
    void onMasterConnect();
    void onMasterDisconnect();
    void onMasterFound();
    void onError(QAbstractSocket::SocketError error);

    void onDataReady();
protected:
    QTcpSocket *_client;
    MasterConfig _config;
};



//
}

#endif //PROJECT_COBOTSYS_COMPUTE_NODE_H
