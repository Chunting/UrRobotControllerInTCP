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

    void runScript(const QString &script_name, std::function<void(bool)> on_slave_reply = nullptr);

    BackgroundServer &getMaster();

protected:
    void onScriptFinish();
    void onSlaveJson(const QJsonObject &json);
    void onSlaveConnect(const QString &slave_name);
    void onSlaveDisconnect(const QString &slave_name);

protected:
    BackgroundServer *_master;

    struct SlaveTaskView {
        bool is_running;

        SlaveTaskView();
    };

    std::map<QString, SlaveTaskView> _views;
};

//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H
