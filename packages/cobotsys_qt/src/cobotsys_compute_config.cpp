//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_compute_config.h"



namespace cobotsys {
namespace server {

QHostAddress localIPv4(){
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for (auto &address : list) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
            return address;
    }
    return QHostAddress(QHostAddress::LocalHost);
}

CONFIG::CONFIG(){
    address = localIPv4();
    port = 45455;
}


//
}
}