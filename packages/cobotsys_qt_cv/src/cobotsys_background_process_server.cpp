//
// Created by 潘绪洋 on 17-2-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_background_process_server.h"


namespace cobotsys {


BackgroundProcessServer::SlaveTaskView::SlaveTaskView(){
    is_running = false;
}


BackgroundProcessServer::BackgroundProcessServer(QObject *parent) : QObject(parent){
    _server = new BackgroundServer(this);
    _server->setJsonHandler(this, &BackgroundProcessServer::onClientJson);
    connect(_server, &BackgroundServer::clientConnected, this, &BackgroundProcessServer::onClientConnect);
    connect(_server, &BackgroundServer::clientDisconnected, this, &BackgroundProcessServer::onClientDisconnect);
}


BackgroundProcessServer::~BackgroundProcessServer(){
}

void BackgroundProcessServer::runScript(const QString &script_name, std::function<void(bool)> on_slave_reply){
    QJsonObject json;
    json[JSON_COMMAND_KEY] = BACK_CMD_RUN_SCRIPT;
    json[BACK_KEY_SCRIPT_NAME] = script_name;
    _server->writeJson(json, [=](const cobotsys::JsonReply &reply){
        if (on_slave_reply) {
            on_slave_reply(reply.reply_status == JsonReplyStatus::Success);
        }
    });
}

void BackgroundProcessServer::onScriptFinish(){
}

BackgroundServer &BackgroundProcessServer::getServer(){
    return *_server;
}

void BackgroundProcessServer::onClientJson(const QJsonObject &json){
    QString slave_name = json[JSON_SENDER].toString();
    auto iter = _views.find(slave_name);
    if (iter != _views.end()) {
        iter->second.is_running = json[BACK_KEY_TASK_STATUS].toBool();
    }
}

void BackgroundProcessServer::onClientConnect(const QString &slave_name){
    _views[slave_name] = SlaveTaskView();
}

void BackgroundProcessServer::onClientDisconnect(const QString &client_name){
    COBOT_LOG.warning() << "Client Exit: " << client_name;
    _views.erase(client_name);
}

BackgroundServer *BackgroundProcessServer::getServerPtr(){
    return _server;
}




//
}