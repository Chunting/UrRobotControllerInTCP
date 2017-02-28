//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QUuid>
#include "cobotsys_background_json_client.h"


namespace cobotsys {
using namespace distributed_system;

BackgroundJsonClient::BackgroundJsonClient(QObject *parent) : ComputeNode(parent){
    _num_debug_inc = 0;
    _instance_id = QUuid::createUuid().toString();
    _decoder = std::make_shared<MessageDecoder>([=](const Message &m){ processMessage(m); });
    _json_callback_manager = std::make_shared<JsonCallbackManager>(
            [=](const QJsonObject &j){ writeJson(j); }, _instance_id);

    registerCommandHandler(BACK_GET_SLAVE_NAME, this, &BackgroundJsonClient::cmdGetSlaveName);
}

void BackgroundJsonClient::processData(const QByteArray &ba){
    _decoder->decode(ba);
}

void BackgroundJsonClient::processConnect(){
    writeData("Hello World!");
}

void BackgroundJsonClient::processDisconnect(){
}

void BackgroundJsonClient::processMessage(const Message &m){
    if (m.getType() == MessageType::Utf8BasedJSON) {
        QJsonParseError jsonParseError;
        QByteArray json(m.getContent(), m.getContentLength());
        auto jsonDoc = QJsonDocument::fromJson(json, &jsonParseError);
        if (jsonDoc.isNull()) {
            COBOT_LOG.warning() << "JSON: " << jsonParseError.errorString();
        } else {
            auto jsonObject = jsonDoc.object();
            processJson(jsonObject);
        }
    }
}

void BackgroundJsonClient::processJson(const QJsonObject &json){
    _json_callback_manager->processJson(json);
}

void BackgroundJsonClient::writeJson(const QJsonObject &json){
    _socket->write(MessageEncoder::genJsonMessage(json).getData());
    COBOT_LOG.info() << "Client JSON: " << json;
}

void BackgroundJsonClient::cmdGetSlaveName(const QJsonObject &json){
    auto rcmd = json;

    rcmd[BACK_KEY_SLAVE_NAME] = _node_name;
    rcmd[BACK_KEY_SLAVE_INSTANCE_ID] = _instance_id;
    rcmd[JSON_REPLY] = "";
    replyJson(rcmd);
}

void BackgroundJsonClient::replyJson(const QJsonObject &json){
    auto rejs = json;
    rejs.remove(JSON_SENDER);
    rejs.remove(JSON_COMMAND_KEY);
    rejs[JSON_REPLY] = "";
    writeJson(rejs);
}

void BackgroundJsonClient::registerCommandHandler(const QString &command, std::function<void(const QJsonObject &)> handler){
    _json_callback_manager->addJsonCommandListener(command, command, handler);
}

//
}