//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_node.h"


namespace cobotsys {

ComputeNode::ComputeNode(QObject *parent) : QObject(parent){
    _socket = new QTcpSocket(this);
    _is_connected = false;
    _re_connect_delay = 100;

    connect(_socket, &QTcpSocket::connected, this, &ComputeNode::onConnect);
    connect(_socket, &QTcpSocket::disconnected, this, &ComputeNode::onDisconnect);
    connect(_socket, &QTcpSocket::hostFound, this, &ComputeNode::onHostFound);
    connect(_socket, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &ComputeNode::onError);
    connect(_socket, &QTcpSocket::readyRead, this, &ComputeNode::onDataReady);
}

ComputeNode::~ComputeNode(){
}

void ComputeNode::connectMaster(const server::CONFIG &config){
    _config = config;
    connectHost();
}

void ComputeNode::onConnect(){
    _is_connected = true;
    COBOT_LOG.notice() << "ComputeNode: " << "Connected";

    processConnect();
}

void ComputeNode::onDisconnect(){
    auto last_connection_status = _is_connected;
    _is_connected = false;
    COBOT_LOG.notice() << "ComputeNode: " << "Disconnected";

    if (last_connection_status)
        processDisconnect();

    connectHost(_re_connect_delay);
}

void ComputeNode::onHostFound(){
    COBOT_LOG.notice() << "ComputeNode: " << "Found Server: " << _socket->peerName();
}

void ComputeNode::onError(QAbstractSocket::SocketError error){
    COBOT_LOG.notice() << "ComputeNode: " << _socket->errorString();

    connectHost(_re_connect_delay);
}

void ComputeNode::onDataReady(){
    auto ba = _socket->readAll();
    if (ba.size()) {
        processData(ba);
    }
}

void ComputeNode::processData(const QByteArray &ba){
    COBOT_LOG.info() << "RECV [" << std::setw(5) << ba.size() << "] " << ba.constData();
}

void ComputeNode::writeData(const QByteArray &ba){
    if (_socket) {
        _socket->write(ba);
    }
}

void ComputeNode::connectHost(int delayMSec){
    if (_is_connected)
        return;

    auto doCONNECT = [=](){
        _socket->connectToHost(_config.address, _config.port, QIODevice::ReadWrite);
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

void ComputeNode::setNodeName(const QString &name){
    _node_name = name;
}

//
}