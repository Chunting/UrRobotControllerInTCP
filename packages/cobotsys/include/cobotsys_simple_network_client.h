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


class SimpleNetworkClient : public QObject {
Q_OBJECT
public:

public:
    SimpleNetworkClient(QObject* parent = nullptr);
    ~SimpleNetworkClient();

    void connectHost(const server::CONFIG& config = server::CONFIG());
    void writeData(const QByteArray& ba);
    void setNodeName(const QString& name);

protected:
    void onConnect();
    void onDisconnect();
    void onHostFound();
    void onError(QAbstractSocket::SocketError error);

    void onDataReady();

    void doConnectHost(int delayMSec = 0);

protected:
    virtual void processData(const QByteArray& ba);
    virtual void processConnect();
    virtual void processDisconnect();


protected:
    QTcpSocket* _socket;
    server::CONFIG _config;
    bool _is_connected;
    int _re_connect_delay;
    QString _node_name;
};



//
}

#endif //PROJECT_COBOTSYS_COMPUTE_NODE_H
