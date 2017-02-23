//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_MESSAGE_SERVER_H
#define PROJECT_COBOTSYS_MESSAGE_SERVER_H

#include <QObject>
#include <QTcpServer>
#include <QUdpSocket>
#include <QTcpSocket>
#include <QTimer>
#include <QCoreApplication>
#include <memory>
#include <cobotsys_qt.h>
#include <cobotsys_logger.h>

namespace cobotsys {

namespace distributed_system {


class MessageServer : public QObject {
Q_OBJECT
public:
    class CONFIG {
    public:
        CONFIG();

        QHostAddress groupAddress;
        uint16_t port;
        int ttl;
    };
public:
    ~MessageServer();

    static void lanuchServer(const CONFIG &conf = CONFIG());

protected:
    void processPendingDatagrams();

private:
    MessageServer(const CONFIG &conf, QObject *parent = nullptr);

    class MessageServerImpl;
    MessageServerImpl *_priv;
};
}
}


#endif //PROJECT_COBOTSYS_MESSAGE_SERVER_H
