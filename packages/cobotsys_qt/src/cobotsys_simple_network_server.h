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


class SimpleNetworkServer : public QObject {
Q_OBJECT
public:
    SimpleNetworkServer(QObject *parent = nullptr);
    ~SimpleNetworkServer();


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
    struct TCPLink {
        QTcpSocket *tcp_socket;
        TCPLink();
        ~TCPLink();
    };

    std::shared_ptr<TCPLink> getLink();
    void deleteTCPLink(std::shared_ptr<TCPLink> link);


protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);
    virtual void processClientConnect(QTcpSocket *tcpSocket);
    virtual void processClientDisconnect(QTcpSocket *tcpSocket);
protected:
    QTcpServer *_server;
    server::CONFIG _config;
    std::map<void *, std::shared_ptr<TCPLink> > _links;
};

//
}

#endif //PROJECT_COBOTSYS_COMPUTE_MASTER_H
