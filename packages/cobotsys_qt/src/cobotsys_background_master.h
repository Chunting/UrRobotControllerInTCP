//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_MASTER_H

#include <cobotsys_json_callback_manager.h>
#include <cobotsys_compute_master.h>
#include <cobotsys_message.h>
#include <chrono>
#include <functional>
#include <cobotsys_background_command_config.h>

namespace cobotsys {
using namespace distributed_system;

class BackgroundMaster : public ComputeMaster {
Q_OBJECT


public:

    class BackgroundSlaveView {
    public:
        BackgroundSlaveView(std::function<void(const QJsonObject &, BackgroundSlaveView &)> jsonHandler);


        QString slave_name;
        QString slave_instance_id;

        friend class BackgroundMaster;
    protected:
        void processData(const QByteArray &ba);
        void processMessage(const Message &m);

    protected:
        std::shared_ptr<MessageDecoder> _decoder;
        std::function<void(const QJsonObject &, BackgroundSlaveView &)> _json_handler;
    };

public:
    BackgroundMaster(QObject *parent = nullptr);
    ~BackgroundMaster();


    void writeJson(const QJsonObject &json, std::function<void(const cobotsys::JsonReply &)> on_slave_reply);

protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);
    virtual void processClientConnect(QTcpSocket *tcpSocket);

    void processJson(const QJsonObject &json, QTcpSocket *link, BackgroundSlaveView &view);

    static void directWriteJson(QTcpSocket *clientLink, const QJsonObject &json);


protected:


protected:
    void createClientView(QTcpSocket *clientLink);


    void cmdGetName(QTcpSocket *clientLink);

protected:
    QString _instance_id;
    std::map<QTcpSocket *, std::shared_ptr<BackgroundSlaveView> > _slaves;

    struct CallbackTracker {
        std::chrono::high_resolution_clock::time_point send_time;
        QJsonObject send_data;
        QTcpSocket *send_link;

        std::function<void(const cobotsys::JsonReply &)> callback;

        double calcTimeElapsed() const;
    };

    std::map<QString, CallbackTracker> _jcmd_history;

protected:
    void writeJson(QTcpSocket *clientLink, const QJsonObject &json,
                   std::function<void(const cobotsys::JsonReply &)> on_slave_reply);
};


//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_MASTER_H
