//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_background_process_master.h"


namespace cobotsys {


BackgroundProcessMaster::SlaveTaskView::SlaveTaskView(){
    is_running = false;
}


BackgroundProcessMaster::BackgroundProcessMaster(QObject *parent) : QObject(parent){
    _master = new BackgroundMaster(this);
    _master->setJsonHandler(this, &BackgroundProcessMaster::onSlaveJson);
    connect(_master, &BackgroundMaster::clientConnected, this, &BackgroundProcessMaster::onSlaveConnect);
    connect(_master, &BackgroundMaster::clientDisconnected, this, &BackgroundProcessMaster::onSlaveDisconnect);
}


BackgroundProcessMaster::~BackgroundProcessMaster(){
}

void BackgroundProcessMaster::runScript(const QString &script_name, std::function<void(bool)> on_slave_reply){
    QJsonObject json;
    json[JSON_COMMAND_KEY] = BACK_CMD_RUN_SCRIPT;
    json[BACK_KEY_SCRIPT_NAME] = script_name;
    _master->writeJson(json, [=](const cobotsys::JsonReply &reply){
        if (on_slave_reply) {
            on_slave_reply(reply.reply_status == JsonReplyStatus::Success);
        }
    });
}

void BackgroundProcessMaster::onScriptFinish(){
}

BackgroundMaster &BackgroundProcessMaster::getMaster(){
    return *_master;
}

void BackgroundProcessMaster::onSlaveJson(const QJsonObject &json){
    QString slave_name = json[JSON_SENDER].toString();
    auto iter = _views.find(slave_name);
    if (iter != _views.end()) {
        iter->second.is_running = json[BACK_KEY_TASK_STATUS].toBool();
    }
}

void BackgroundProcessMaster::onSlaveConnect(const QString &slave_name){
    _views[slave_name] = SlaveTaskView();
}

void BackgroundProcessMaster::onSlaveDisconnect(const QString &slave_name){
    COBOT_LOG.warning() << "Client Exit: " << slave_name;
    _views.erase(slave_name);
}




//
}