//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_WORKER_H
#define PROJECT_COBOTSYS_BACKGROUND_WORKER_H

#include <cobotsys_compute_node.h>

namespace cobotsys {


class BackgroundSlave : public ComputeNode {
Q_OBJECT
public:
    BackgroundSlave(QObject *parent = nullptr);

protected:
    virtual void processData(const QByteArray &ba);
    virtual void processConnect();
    virtual void processDisconnect();

protected:
    uint32_t _num_debug_inc;
};

//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_WORKER_H
