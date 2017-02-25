//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_node.h"


namespace cobotsys {

ComputeNode::ComputeNode(QObject *parent) : QObject(parent){
    _client = new QTcpSocket(this);

    connect(_client, &QTcpSocket::connected, this, &ComputeNode::onMasterConnect);
    connect(_client, &QTcpSocket::disconnected, this, &ComputeNode::onMasterDisconnect);
    connect(_client, &QTcpSocket::hostFound, this, &ComputeNode::onMasterFound);
    connect(_client, static_cast<void (QTcpSocket::*)(QAbstractSocket::SocketError)>(&QTcpSocket::error), this,
            &ComputeNode::onError);
    connect(_client, &QTcpSocket::readyRead, this, &ComputeNode::onDataReady);
}

ComputeNode::~ComputeNode(){
}

void ComputeNode::connectMaster(const ComputeNode::MasterConfig &config){
    _client->connectToHost(config.address, config.port, QIODevice::ReadWrite);
}

void ComputeNode::onMasterConnect(){
}

void ComputeNode::onMasterDisconnect(){
}

void ComputeNode::onMasterFound(){
}

void ComputeNode::onError(QAbstractSocket::SocketError error){
}

void ComputeNode::onDataReady(){
}


//
}