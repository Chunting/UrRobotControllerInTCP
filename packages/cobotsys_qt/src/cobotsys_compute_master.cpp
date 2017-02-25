//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_master.h"

namespace cobotsys {
ComputeMaster::TCPLink::TCPLink(){
    tcp_socket = nullptr;
}

ComputeMaster::TCPLink::~TCPLink(){
    if (tcp_socket) {
        tcp_socket->deleteLater();
    }
}
}


namespace cobotsys {


ComputeMaster::ComputeMaster(QObject *parent) : QObject(parent){
    _server = new QTcpServer(this);
    connect(_server, &QTcpServer::newConnection, this, &ComputeMaster::onNewConnection);
}

void ComputeMaster::onNewConnection(){
    auto tcp_socket = _server->nextPendingConnection();
    connect(tcp_socket, &QTcpSocket::connected, this, &ComputeMaster::onClientConnect);
    connect(tcp_socket, &QTcpSocket::disconnected, this, &ComputeMaster::onClientDisconnect);
    connect(tcp_socket, &QTcpSocket::hostFound, this, &ComputeMaster::onClientFound);
    connect(tcp_socket, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &ComputeMaster::onClientError);
    connect(tcp_socket, &QTcpSocket::readyRead, this, &ComputeMaster::onClientDataReady);

    auto link = std::make_shared<TCPLink>();

    link->tcp_socket = tcp_socket;

    _links[tcp_socket] = link;
}


ComputeMaster::~ComputeMaster(){
}

bool ComputeMaster::lanuchMaster(const server::CONFIG &config){
    if (_server->listen(config.address, config.port)) {
        COBOT_LOG.notice() << "ComputeMaster, Lanuch Success.";
        return true;
    }
    return false;
}

void ComputeMaster::onClientConnect(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Connect.";
    }
}

void ComputeMaster::onClientDisconnect(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Disconnect.";

        deleteTCPLink(pLink);
    }
}

void ComputeMaster::onClientFound(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Found.";
    }
}

void ComputeMaster::onClientError(QAbstractSocket::SocketError error){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client, " << pLink->tcp_socket->errorString();
    }
}

void ComputeMaster::onClientDataReady(){
    auto pLink = getLink();
    if (pLink) {
        auto ba = pLink->tcp_socket->readAll();
        processClientData(pLink->tcp_socket, ba);
    }
}

std::shared_ptr<ComputeMaster::TCPLink> ComputeMaster::getLink(){
    auto iter = _links.find(sender());
    if (iter != _links.end())
        return iter->second;
    return nullptr;
}

void ComputeMaster::deleteTCPLink(std::shared_ptr<ComputeMaster::TCPLink> link){
    _links.erase(link->tcp_socket);
}

void ComputeMaster::processClientData(QTcpSocket *clientLink, const QByteArray &ba){
    COBOT_LOG.info() << clientLink << ": " << ba.constData();
    clientLink->write(ba);
}



//
}