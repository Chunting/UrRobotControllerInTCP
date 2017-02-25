//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_MASTER_H

#include <cobotsys_compute_master.h>

namespace cobotsys {


class BackgroundMaster : public ComputeMaster {
Q_OBJECT
public:
    BackgroundMaster(QObject *parent);
    ~BackgroundMaster();


protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);

protected:


};


//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_MASTER_H
