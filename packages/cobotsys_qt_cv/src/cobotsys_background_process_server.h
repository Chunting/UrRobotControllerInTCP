//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H

#include <QObject>
#include <cobotsys_background_server.h>

namespace cobotsys {

class BackgroundProcessServer : public QObject {
Q_OBJECT
public:
    BackgroundProcessServer(QObject *parent = nullptr);
    ~BackgroundProcessServer();

    void runScript(const QString &script_name, std::function<void(bool)> on_client_reply = nullptr);
    void stopScript(std::function<void(bool)> on_client_reply = nullptr);

    BackgroundServer &getServer();
    BackgroundServer *getServerPtr();

Q_SIGNALS:
    void clientTaskChanged(const QString& client, bool is_task_running);

protected:
    void onScriptFinish();
    void onClientJson(const QJsonObject &json);
    void onClientConnect(const QString &client_name);
    void onClientDisconnect(const QString &client_name);

protected:
    BackgroundServer *_server;

    struct SlaveTaskView {
        bool is_running;

        SlaveTaskView();
    };

    std::map<QString, SlaveTaskView> _views;
};

//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H
