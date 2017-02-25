//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_background_slave.h"


namespace cobotsys {


BackgroundSlave::BackgroundSlave(QObject *parent) : ComputeNode(parent){
    _num_debug_inc = 0;
}

void BackgroundSlave::processData(const QByteArray &ba){
    writeData(QString::number(_num_debug_inc++).toLocal8Bit());
    COBOT_LOG.info() << "Reply: " << ba.constData();
}

void BackgroundSlave::processConnect(){
    writeData("Hello World!");
}

void BackgroundSlave::processDisconnect(){
}

//
}