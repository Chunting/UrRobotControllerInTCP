//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_COMPUTE_MASTER_H
#define PROJECT_COBOTSYS_COMPUTE_MASTER_H

#include <map>
#include <deque>
#include <memory>
#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <cobotsys_compute_config.h>
#include <cobotsys_qt.h>

namespace cobotsys {


class ComputeMaster : public QObject {
Q_OBJECT
public:
    ComputeMaster(QObject *parent = nullptr);
    ~ComputeMaster();


    bool lanuchMaster(const server::CONFIG &config = server::CONFIG());

protected:
    void onNewConnection();


protected:
    void onClientConnect();
    void onClientDisconnect();
    void onClientFound();
    void onClientError(QAbstractSocket::SocketError error);
    void onClientDataReady();
protected:
    struct TcpLink {
        QTcpSocket *tcp_socket;
        TcpLink();
    };
protected:
    QTcpServer *_server;
    server::CONFIG _config;
    std::map<void *, std::shared_ptr<TcpLink> > _links;
};

//
}

#endif //PROJECT_COBOTSYS_COMPUTE_MASTER_H
