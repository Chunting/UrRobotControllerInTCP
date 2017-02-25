//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QUuid>
#include "cobotsys_background_slave.h"


namespace cobotsys {
using namespace distributed_system;

BackgroundSlave::BackgroundSlave(QObject *parent) : ComputeNode(parent){
    _num_debug_inc = 0;
    _instance_id = QUuid::createUuid().toString();
    _decoder = std::make_shared<MessageDecoder>([=](const Message &m){ processMessage(m); });
    _json_callback_manager = std::make_shared<JsonCallbackManager>(
            [=](const QJsonObject &j){ writeJson(j); }, _instance_id);

    _json_callback_manager->addJsonCommandListener(
            BACK_GET_SLAVE_NAME, BACK_GET_SLAVE_NAME, [=](const QJsonObject &j){ cmdGetSlaveName(j); });
}

void BackgroundSlave::processData(const QByteArray &ba){
    _decoder->decode(ba);
}

void BackgroundSlave::processConnect(){
    writeData("Hello World!");
}

void BackgroundSlave::processDisconnect(){
}

void BackgroundSlave::processMessage(const Message &m){
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

void BackgroundSlave::processJson(const QJsonObject &json){
    _json_callback_manager->processJson(json);
}

void BackgroundSlave::writeJson(const QJsonObject &json){
    _client->write(MessageEncoder::genJsonMessage(json).getData());
}

void BackgroundSlave::cmdGetSlaveName(const QJsonObject &json){
    auto rcmd = json;

    rcmd[BACK_KEY_SLAVE_NAME] = _instance_id;
    replyJson(rcmd);
}

void BackgroundSlave::replyJson(const QJsonObject &json){
    auto rejs = json;
    rejs.remove(JSON_SENDER);
    rejs.remove(JSON_COMMAND_KEY);
    writeJson(rejs);
}

//
}