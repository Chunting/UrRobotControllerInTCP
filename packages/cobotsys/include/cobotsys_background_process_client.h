//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BACKGROUND_PROCESS_CLIENT_H
#define PROJECT_COBOTSYS_BACKGROUND_PROCESS_CLIENT_H


#include <QObject>
#include <cobotsys_background_json_client.h>
#include "cobotsys_background_task.h"


namespace cobotsys {


class BackgroundProcessClient : public QObject {
Q_OBJECT
public:
    BackgroundProcessClient(QObject* parent = nullptr);
    ~BackgroundProcessClient();


    BackgroundJsonClient& getSlave();


    void setScriptConfigFile(const QString& filename);
protected:

    void onRunScript(const QJsonObject& json);
    void onStopScript(const QJsonObject& json);

    bool loadScriptSetting(const QString& settingName);

    void onTaskFinish();
protected:
    QString _script_file_name;
    BackgroundJsonClient _slave;
    BackgroundTask* _task;

    BackgroundTaskSettings _task_setting;
};

//
}

#endif //PROJECT_COBOTSYS_BACKGROUND_PROCESS_CLIENT_H
