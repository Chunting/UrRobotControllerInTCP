//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_WORKER_H
#define PROJECT_COBOTSYS_BACKGROUND_WORKER_H

#include <cobotsys_compute_node.h>
#include <cobotsys_message.h>
#include <cobotsys_json_callback_manager.h>
#include <cobotsys_background_command_config.h>

namespace cobotsys {
using namespace distributed_system;

class BackgroundSlave : public ComputeNode {
Q_OBJECT

public:
    BackgroundSlave(QObject *parent = nullptr);


    void registerCommandHandler(const QString &command, std::function<void(const QJsonObject &)> handler);
protected:
    virtual void processData(const QByteArray &ba);
    virtual void processConnect();
    virtual void processDisconnect();

protected:
    void processMessage(const Message &m);
    void processJson(const QJsonObject &json);
    void writeJson(const QJsonObject &json);
    void replyJson(const QJsonObject &json);
protected:
    void cmdGetSlaveName(const QJsonObject &json);

protected:
    uint32_t _num_debug_inc;
    QString _instance_id;
    std::shared_ptr<MessageDecoder> _decoder;
    std::shared_ptr<JsonCallbackManager> _json_callback_manager;
};

//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_WORKER_H
