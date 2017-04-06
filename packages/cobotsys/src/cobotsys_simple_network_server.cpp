//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_simple_network_server.h"

namespace cobotsys {
SimpleNetworkServer::TCPLink::TCPLink() {
    tcp_socket = nullptr;
}

SimpleNetworkServer::TCPLink::~TCPLink() {
    if (tcp_socket) {
        tcp_socket->deleteLater();
    }
}
}


namespace cobotsys {


SimpleNetworkServer::SimpleNetworkServer(QObject* parent) : QObject(parent) {
    _server = new QTcpServer(this);
    connect(_server, &QTcpServer::newConnection, this, &SimpleNetworkServer::onNewConnection);
}

void SimpleNetworkServer::onNewConnection() {
    auto tcp_socket = _server->nextPendingConnection();
    connect(tcp_socket, &QTcpSocket::connected, this, &SimpleNetworkServer::onClientConnect);
    connect(tcp_socket, &QTcpSocket::disconnected, this, &SimpleNetworkServer::onClientDisconnect);
    connect(tcp_socket, &QTcpSocket::hostFound, this, &SimpleNetworkServer::onClientFound);
    connect(tcp_socket, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &SimpleNetworkServer::onClientError);
    connect(tcp_socket, &QTcpSocket::readyRead, this, &SimpleNetworkServer::onClientDataReady);

    auto link = std::make_shared<TCPLink>();

    link->tcp_socket = tcp_socket;

    _links[tcp_socket] = link;

    processClientConnect(tcp_socket);
}


SimpleNetworkServer::~SimpleNetworkServer() {
}

bool SimpleNetworkServer::lanuchMaster(const server::CONFIG& config) {
    if (_server->listen(config.address, config.port)) {
        _config = config;
        COBOT_LOG.notice() << "SimpleNetworkServer, Lanuch Success. [" << config.address.toString()
                           << ":" << config.port << "]";
        return true;
    }
    return false;
}

void SimpleNetworkServer::onClientConnect() {
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Server: " << "Client Connect.";
    }
}

void SimpleNetworkServer::onClientDisconnect() {
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Server: " << "Client Disconnect.";

        deleteTCPLink(pLink);
        processClientDisconnect(pLink->tcp_socket);
    }
}

void SimpleNetworkServer::onClientFound() {
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Server: " << "Client Found.";
    }
}

void SimpleNetworkServer::onClientError(QAbstractSocket::SocketError error) {
    auto pLink = getLink();
    if (pLink) {
        COBOT_LOG.notice() << "Server: " << "Error: " << pLink->tcp_socket->errorString();
    }
}

void SimpleNetworkServer::onClientDataReady() {
    auto pLink = getLink();
    if (pLink) {
        auto ba = pLink->tcp_socket->readAll();
        processClientData(pLink->tcp_socket, ba);
    }
}

std::shared_ptr<SimpleNetworkServer::TCPLink> SimpleNetworkServer::getLink() {
    auto iter = _links.find(sender());
    if (iter != _links.end())
        return iter->second;
    return nullptr;
}

void SimpleNetworkServer::deleteTCPLink(std::shared_ptr<SimpleNetworkServer::TCPLink> link) {
    _links.erase(link->tcp_socket);
}

void SimpleNetworkServer::processClientData(QTcpSocket* clientLink, const QByteArray& ba) {
    COBOT_LOG.info() << clientLink << ": " << ba.constData();
    clientLink->write(ba);
}

void SimpleNetworkServer::processClientConnect(QTcpSocket* tcpSocket) {
}

void SimpleNetworkServer::processClientDisconnect(QTcpSocket* tcpSocket) {
}



//
}