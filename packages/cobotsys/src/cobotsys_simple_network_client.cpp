//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_simple_network_client.h"


namespace cobotsys {

SimpleNetworkClient::SimpleNetworkClient(QObject* parent) : QObject(parent) {
    _socket = new QTcpSocket(this);
    _is_connected = false;
    _re_connect_delay = 100;

    connect(_socket, &QTcpSocket::connected, this, &SimpleNetworkClient::onConnect);
    connect(_socket, &QTcpSocket::disconnected, this, &SimpleNetworkClient::onDisconnect);
    connect(_socket, &QTcpSocket::hostFound, this, &SimpleNetworkClient::onHostFound);
    connect(_socket, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &SimpleNetworkClient::onError);
    connect(_socket, &QTcpSocket::readyRead, this, &SimpleNetworkClient::onDataReady);
}

SimpleNetworkClient::~SimpleNetworkClient() {
}

void SimpleNetworkClient::connectHost(const server::CONFIG& config) {
    _config = config;
    doConnectHost();
}

void SimpleNetworkClient::onConnect() {
    _is_connected = true;
    COBOT_LOG.notice() << "SimpleNetworkClient: " << "Connected";

    processConnect();
}

void SimpleNetworkClient::onDisconnect() {
    auto last_connection_status = _is_connected;
    _is_connected = false;
    COBOT_LOG.notice() << "SimpleNetworkClient: " << "Disconnected";

    if (last_connection_status)
        processDisconnect();

    doConnectHost(_re_connect_delay);
}

void SimpleNetworkClient::onHostFound() {
    COBOT_LOG.notice() << "SimpleNetworkClient: " << "Found Server: " << _socket->peerName();
}

void SimpleNetworkClient::onError(QAbstractSocket::SocketError error) {
    COBOT_LOG.notice() << "SimpleNetworkClient: " << _socket->errorString();

    doConnectHost(_re_connect_delay);
}

void SimpleNetworkClient::onDataReady() {
    auto ba = _socket->readAll();
    if (ba.size()) {
        processData(ba);
    }
}

void SimpleNetworkClient::processData(const QByteArray& ba) {
    COBOT_LOG.info() << "RECV [" << std::setw(5) << ba.size() << "] " << ba.constData();
}

void SimpleNetworkClient::writeData(const QByteArray& ba) {
    if (_socket) {
        _socket->write(ba);
    }
}

void SimpleNetworkClient::doConnectHost(int delayMSec) {
    if (_is_connected)
        return;

    auto doCONNECT = [=]() {
        _socket->connectToHost(_config.address, _config.port, QIODevice::ReadWrite);
    };

    if (delayMSec > 0)
        QTimer::singleShot(delayMSec, doCONNECT);
    else
        doCONNECT();
}

void SimpleNetworkClient::processConnect() {
}

void SimpleNetworkClient::processDisconnect() {
}

void SimpleNetworkClient::setNodeName(const QString& name) {
    _node_name = name;
}

//
}