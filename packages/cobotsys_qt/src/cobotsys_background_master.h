//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_MASTER_H

#include <cobotsys_compute_master.h>
#include <cobotsys_json_callback_manager.h>

namespace cobotsys {


class BackgroundMaster : public ComputeMaster {
Q_OBJECT
public:
    BackgroundMaster(QObject *parent);
    ~BackgroundMaster();


protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);

protected:
    std::shared_ptr<JsonCallbackManager> _callback_manager;
    QString _instance_id;
};


//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_MASTER_H
