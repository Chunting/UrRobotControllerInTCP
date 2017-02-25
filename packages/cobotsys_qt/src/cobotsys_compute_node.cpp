//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_node.h"


namespace cobotsys {

ComputeNode::ComputeNode(QObject *parent) : QObject(parent){
    _client = new QTcpSocket(this);
    _is_connected = false;
    _re_connect_delay = 100;

    connect(_client, &QTcpSocket::connected, this, &ComputeNode::onMasterConnect);
    connect(_client, &QTcpSocket::disconnected, this, &ComputeNode::onMasterDisconnect);
    connect(_client, &QTcpSocket::hostFound, this, &ComputeNode::onMasterFound);
    connect(_client, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &ComputeNode::onError);
    connect(_client, &QTcpSocket::readyRead, this, &ComputeNode::onDataReady);
}

ComputeNode::~ComputeNode(){
}

void ComputeNode::connectMaster(const server::CONFIG &config){
    _config = config;
    connectHost();
}

void ComputeNode::onMasterConnect(){
    _is_connected = true;
    COBOT_LOG.notice() << "ComputeNode: " << "Connected";
    Q_EMIT masterConnected();

    processConnect();
}

void ComputeNode::onMasterDisconnect(){
    auto last_connection_status = _is_connected;
    _is_connected = false;
    COBOT_LOG.notice() << "ComputeNode: " << "Disconnected";
    Q_EMIT masterDisconnected();

    if (last_connection_status)
        processDisconnect();

    connectHost(_re_connect_delay);
}

void ComputeNode::onMasterFound(){
    COBOT_LOG.notice() << "ComputeNode: " << "Found: " << _client->peerName();
}

void ComputeNode::onError(QAbstractSocket::SocketError error){
    COBOT_LOG.notice() << "ComputeNode: " << _client->errorString();

    connectHost(_re_connect_delay);
}

void ComputeNode::onDataReady(){
    auto ba = _client->readAll();
    if (ba.size()) {
        processData(ba);
    }
}

void ComputeNode::processData(const QByteArray &ba){
    COBOT_LOG.info() << "RECV [" << std::setw(5) << ba.size() << "] " << ba.constData();
}

void ComputeNode::writeData(const QByteArray &ba){
    if (_client) {
        _client->write(ba);
    }
}

void ComputeNode::connectHost(int delayMSec){
    if (_is_connected)
        return;

    auto doCONNECT = [=](){
        _client->connectToHost(_config.address, _config.port, QIODevice::ReadWrite);
    };

    if (delayMSec > 0)
        QTimer::singleShot(delayMSec, doCONNECT);
    else
        doCONNECT();
}

void ComputeNode::processConnect(){
}

void ComputeNode::processDisconnect(){
}

//
}