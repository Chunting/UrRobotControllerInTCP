//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "cobotsys_message_server.h"

namespace cobotsys {

namespace distributed_system {

class MessageServer::MessageServerImpl : public QObject {
public:
    QUdpSocket *udpSocket;
    QTimer *udpMulticastTimer;
    uint32_t udpMessageNo;
    CONFIG config;
public:
    MessageServerImpl(const CONFIG &conf, QObject *parent) : QObject(parent){
        config = conf;
        udpMessageNo = 0;

        udpSocket = new QUdpSocket(this);
        udpSocket->bind(QHostAddress::AnyIPv4, conf.port, QUdpSocket::ShareAddress);
        udpSocket->joinMulticastGroup(conf.groupAddress);
        udpSocket->setSocketOption(QAbstractSocket::MulticastTtlOption, conf.ttl);
        connect(udpSocket, &QUdpSocket::readyRead, this, &MessageServerImpl::processPendingDatagrams);

        udpMulticastTimer = new QTimer(this);
        udpMulticastTimer->setInterval(100);
        connect(udpMulticastTimer, &QTimer::timeout, this, &MessageServerImpl::sendDatagram);
        udpMulticastTimer->start();
    }

    void sendDatagram(){
        QByteArray datagram = "Multicast message " + QByteArray::number(udpMessageNo++) + ", PID: " +
                              QByteArray::number(QCoreApplication::applicationPid());
        udpSocket->writeDatagram(datagram.data(), datagram.size(), config.groupAddress, config.port);
    }

    void processPendingDatagrams(){
        while (udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());
            udpSocket->readDatagram(datagram.data(), datagram.size());
            COBOT_LOG.info() << tr("Received datagram: \"%1\"").arg(datagram.data());
        }
    }
};

MessageServer::CONFIG::CONFIG(){
    groupAddress = QHostAddress("239.255.43.21");
    port = 45455;
    ttl = 2;
}
}
}


namespace cobotsys {

namespace distributed_system {


MessageServer::MessageServer(const MessageServer::CONFIG &conf, QObject *parent) : QObject(parent){
    _priv = new MessageServerImpl(conf, this);
}


MessageServer::~MessageServer(){
}

void MessageServer::lanuchServer(const MessageServer::CONFIG &conf){
    static std::shared_ptr<MessageServer> messageServer;
    if (messageServer == nullptr) {
        messageServer = std::shared_ptr<MessageServer>(new MessageServer(conf));
    }
}

void MessageServer::processPendingDatagrams(){
}
}
}