//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QUuid>
#include "cobotsys_background_master.h"

namespace cobotsys {

BackgroundMaster::BackgroundSlaveView::BackgroundSlaveView(const QString &instancId){
    _decoder = std::make_shared<MessageDecoder>([=](const Message &m){ processMessage(m); });
    _callback_manager = std::make_shared<JsonCallbackManager>(instancId);
}

void BackgroundMaster::BackgroundSlaveView::processData(const QByteArray &ba){
    _decoder->decode(ba);
}

void BackgroundMaster::BackgroundSlaveView::processMessage(const distributed_system::Message &m){
    if (m.getType() == MessageType::Utf8BasedJSON) {
        QJsonParseError jsonParseError;
        QByteArray json(m.getContent(), m.getContentLength());
        auto jsonDoc = QJsonDocument::fromJson(json, &jsonParseError);
        if (jsonDoc.isNull()) {
            COBOT_LOG.warning() << "JSON: " << jsonParseError.errorString();
        } else {
            auto jsonObject = jsonDoc.object();
            _callback_manager->processJson(jsonObject);
        }
    }
}


//
}


namespace cobotsys {


BackgroundMaster::BackgroundMaster(QObject *parent) : ComputeMaster(parent){
    _instance_id = QUuid::createUuid().toString();

}


BackgroundMaster::~BackgroundMaster(){
}

void BackgroundMaster::processClientData(QTcpSocket *clientLink, const QByteArray &ba){
    createClientView(clientLink);

    // 处理数据
    auto viewIter = _slaves.find(clientLink);
    if (viewIter != _slaves.end()) {
        viewIter->second->processData(ba);
    }
}


void BackgroundMaster::createClientView(QTcpSocket *clientLink){
    auto iter = _slaves.find(clientLink);
    if (iter == _slaves.end()) {
        _slaves.insert({clientLink, std::make_shared<BackgroundSlaveView>(_instance_id)});
    }
}





//
}