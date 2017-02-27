//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H
#define PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H

#include <QObject>
#include <cobotsys_background_master.h>

namespace cobotsys {

class BackgroundProcessMaster : public QObject {
Q_OBJECT
public:
    BackgroundProcessMaster(QObject *parent = nullptr);
    ~BackgroundProcessMaster();

    void runScript(const QString &script_name, std::function<void(bool)> on_slave_reply = nullptr);

    BackgroundMaster &getMaster();
Q_SIGNALS:


protected:
    void onScriptFinish();
    void onSlaveJson(const QJsonObject &json);
    void onSlaveConnect(const QString &slave_name);
    void onSlaveDisconnect(const QString &slave_name);

protected:
    BackgroundMaster *_master;

    struct SlaveTaskView {
        bool is_running;

        SlaveTaskView();
    };

    std::map<QString, SlaveTaskView> _views;
};

//
}


#endif //PROJECT_COBOTSYS_BACKGROUND_PROCESS_MASTER_H
