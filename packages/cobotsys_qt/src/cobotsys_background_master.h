//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_MASTER_H

#include <cobotsys_compute_master.h>
#include <cobotsys_json_callback_manager.h>
#include <cobotsys_message.h>

namespace cobotsys {
using namespace distributed_system;

class BackgroundMaster : public ComputeMaster {
Q_OBJECT


public:

    class BackgroundSlaveView {
    public:
        BackgroundSlaveView(std::function<void(const QJsonObject &)> jsonHandler);

        friend class BackgroundMaster;
    protected:
        void processData(const QByteArray &ba);
        void processMessage(const Message &m);

    protected:
        std::shared_ptr<MessageDecoder> _decoder;
        std::function<void(const QJsonObject &)> _json_handler;
    };

public:
    BackgroundMaster(QObject *parent = nullptr);
    ~BackgroundMaster();


protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);
    virtual void processClientConnect(QTcpSocket *tcpSocket);

    void processJson(const QJsonObject &json, QTcpSocket *link);

    static void directWriteJson(QTcpSocket *clientLink, const QJsonObject &json);
protected:
    void createClientView(QTcpSocket *clientLink);


protected:
    QString _instance_id;
    std::map<QTcpSocket *, std::shared_ptr<BackgroundSlaveView> > _slaves;
};


//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_MASTER_H
