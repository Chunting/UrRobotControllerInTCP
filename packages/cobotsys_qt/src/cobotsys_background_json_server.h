//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_MASTER_H

#include <cobotsys_json_callback_manager.h>
#include <cobotsys_compute_node_server.h>
#include <cobotsys_message.h>
#include <chrono>
#include <functional>
#include <cobotsys_background_command_config.h>

namespace cobotsys {
using namespace distributed_system;

class BackgroundJsonServer : public ComputeNodeServer {
Q_OBJECT


public:

    class BackgroundSlaveView {
    public:
        BackgroundSlaveView(std::function<void(const QJsonObject &, BackgroundSlaveView &)> jsonHandler);


        QString slave_name;
        QString slave_instance_id;

        const QString &getName() const{ return slave_name; }

        friend class BackgroundJsonServer;
    protected:
        void processData(const QByteArray &ba);
        void processMessage(const Message &m);

    protected:
        std::shared_ptr<MessageDecoder> _decoder;
        std::function<void(const QJsonObject &, BackgroundSlaveView &)> _json_handler;
    };

public:
    BackgroundJsonServer(QObject *parent = nullptr);
    ~BackgroundJsonServer();


    void writeJson(const QJsonObject &json, std::function<void(const cobotsys::JsonReply &)> on_slave_reply);

    template<class T>
    void writeJson(const QJsonObject &json, T *pObj, void(T::* pFunc)(const cobotsys::JsonReply &)){
        writeJson(json, [=](const cobotsys::JsonReply &r){ (pObj->*pFunc)(r); });
    }


    void setJsonHandler(std::function<void(const QJsonObject &)> general_handler);

    template<class T>
    void setJsonHandler(T *p, void(T::* f)(const QJsonObject &)){
        setJsonHandler([=](const QJsonObject &j){ (p->*f)(j); });
    }


Q_SIGNALS:
    void clientConnected(const QString &name);
    void clientDisconnected(const QString &name);

protected:
    virtual void processClientData(QTcpSocket *clientLink, const QByteArray &ba);
    virtual void processClientConnect(QTcpSocket *tcpSocket);
    virtual void processClientDisconnect(QTcpSocket *tcpSocket);


    static void directWriteJson(QTcpSocket *clientLink, const QJsonObject &json);

protected:
    void processJson(const QJsonObject &json, QTcpSocket *link, BackgroundSlaveView &view);

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

    std::function<void(const QJsonObject &)> _json_handler;

    QTimer *_chk_timer;
protected:
    void writeJson(QTcpSocket *clientLink, const QJsonObject &json,
                   std::function<void(const cobotsys::JsonReply &)> on_slave_reply);

    void callJsonHandler(const QJsonObject &json);

    void timeoutChecker();
};


//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_MASTER_H
