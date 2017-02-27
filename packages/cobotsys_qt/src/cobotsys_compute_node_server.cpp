//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_node_server.h"

namespace cobotsys {
ComputeNodeServer::TCPLink::TCPLink(){
    tcp_socket = nullptr;
}

ComputeNodeServer::TCPLink::~TCPLink(){
    if (tcp_socket) {
        tcp_socket->deleteLater();
    }
}
}


namespace cobotsys {


ComputeNodeServer::ComputeNodeServer(QObject *parent) : QObject(parent){
    _server = new QTcpServer(this);
    connect(_server, &QTcpServer::newConnection, this, &ComputeNodeServer::onNewConnection);
}

void ComputeNodeServer::onNewConnection(){
    auto tcp_socket = _server->nextPendingConnection();
    connect(tcp_socket, &QTcpSocket::connected, this, &ComputeNodeServer::onClientConnect);
    connect(tcp_socket, &QTcpSocket::disconnected, this, &ComputeNodeServer::onClientDisconnect);
    connect(tcp_socket, &QTcpSocket::hostFound, this, &ComputeNodeServer::onClientFound);
    connect(tcp_socket, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &ComputeNodeServer::onClientError);
    connect(tcp_socket, &QTcpSocket::readyRead, this, &ComputeNodeServer::onClientDataReady);

    auto link = std::make_shared<TCPLink>();

    link->tcp_socket = tcp_socket;

    _links[tcp_socket] = link;

    processClientConnect(tcp_socket);
}


ComputeNodeServer::~ComputeNodeServer(){
}

bool ComputeNodeServer::lanuchMaster(const server::CONFIG &config){
    if (_server->listen(config.address, config.port)) {
        COBOT_LOG.notice() << "ComputeNodeServer, Lanuch Success. [" << config.address.toString()
                           << ":" << config.port << "]";
        return true;
    }
    return false;
}

void ComputeNodeServer::onClientConnect(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Connect.";
    }
}

void ComputeNodeServer::onClientDisconnect(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Disconnect.";

        deleteTCPLink(pLink);
        processClientDisconnect(pLink->tcp_socket);
    }
}

void ComputeNodeServer::onClientFound(){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client Found.";
    }
}

void ComputeNodeServer::onClientError(QAbstractSocket::SocketError error){
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Master: " << "Client, " << pLink->tcp_socket->errorString();
    }
}

void ComputeNodeServer::onClientDataReady(){
    auto pLink = getLink();
    if (pLink) {
        auto ba = pLink->tcp_socket->readAll();
        processClientData(pLink->tcp_socket, ba);
    }
}

std::shared_ptr<ComputeNodeServer::TCPLink> ComputeNodeServer::getLink(){
    auto iter = _links.find(sender());
    if (iter != _links.end())
        return iter->second;
    return nullptr;
}

void ComputeNodeServer::deleteTCPLink(std::shared_ptr<ComputeNodeServer::TCPLink> link){
    _links.erase(link->tcp_socket);
}

void ComputeNodeServer::processClientData(QTcpSocket *clientLink, const QByteArray &ba){
    COBOT_LOG.info() << clientLink << ": " << ba.constData();
    clientLink->write(ba);
}

void ComputeNodeServer::processClientConnect(QTcpSocket *tcpSocket){
}

void ComputeNodeServer::processClientDisconnect(QTcpSocket *tcpSocket){
}



//
}