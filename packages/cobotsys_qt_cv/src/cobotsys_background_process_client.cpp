//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_background_process_client.h"


namespace cobotsys {


BackgroundProcessClient::BackgroundProcessClient(QObject *parent) : QObject(parent){

    getSlave().registerCommandHandler(BACK_CMD_RUN_SCRIPT, this, &BackgroundProcessClient::onRunScript);
    _task = new BackgroundTask(this);
    connect(_task, &BackgroundTask::taskFinished, this, &BackgroundProcessClient::onTaskFinish);
}


BackgroundProcessClient::~BackgroundProcessClient(){
}


BackgroundClient &BackgroundProcessClient::getSlave(){
    return _slave;
}

void BackgroundProcessClient::onRunScript(const QJsonObject &json){
    auto reply_json = json;
    auto script_name = json[BACK_KEY_SCRIPT_NAME].toString();
    if (script_name.isEmpty()) {
        reply_json[BACK_KEY_RESULT] = false;
    } else {
        auto result = loadScriptSetting(script_name);
        _task->run(_task_setting);
        reply_json[BACK_KEY_RESULT] = result;
    }
    _slave.replyJson(reply_json);
}

void BackgroundProcessClient::onKillScript(const QJsonObject &json){
    auto reply_json = json;
    _task->stop();
    _slave.replyJson(reply_json);
}

bool BackgroundProcessClient::loadScriptSetting(const QString &settingName){
    std::string setting_str = settingName.toLocal8Bit().constData();
    cv::FileStorage fs(_script_file_name.toLocal8Bit().constData(), cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs[setting_str] >> _task_setting;
        _task_setting.debugPrint(setting_str);
        return true;
    }
    return false;
}

void BackgroundProcessClient::setScriptConfigFile(const QString &filename){
    _script_file_name = filename;
}

void BackgroundProcessClient::onTaskFinish(){
    QJsonObject json;
    json[JSON_COMMAND_KEY] = BACK_CMD_STATUS;
    json[BACK_KEY_TASK_STATUS] = false;
    _slave.writeJson(json);
}



//
}