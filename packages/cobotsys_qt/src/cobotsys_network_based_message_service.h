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
#include <QNetworkInterface>
#include <memory>
#include <cobotsys_qt.h>
#include <cobotsys_logger.h>
#include <sstream>
#include <iomanip>

#include <cobotsys_message.h>

namespace cobotsys {

namespace distributed_system {


class MessageService : public QObject {
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
    ~MessageService();

    static void lanuchService(const CONFIG &conf = CONFIG());
    static void lanuchServiceServer(const CONFIG &conf = CONFIG());

    static void send(const QString &target, const QByteArray &data); // 无管理
private:
    MessageService(const CONFIG &conf, QObject *parent = nullptr);

    class MessageServerImpl;
    MessageServerImpl *_priv;
};
}
}


#endif //PROJECT_COBOTSYS_MESSAGE_SERVER_H
