//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_COMPUTE_CONFIG_H
#define PROJECT_COBOTSYS_COMPUTE_CONFIG_H

#include <QHostAddress>
#include <QNetworkInterface>

namespace cobotsys {


namespace server {


struct CONFIG {
    QHostAddress address;
    uint16_t port;

    CONFIG();
};


QHostAddress localIPv4();
}
}

#endif //PROJECT_COBOTSYS_COMPUTE_CONFIG_H
