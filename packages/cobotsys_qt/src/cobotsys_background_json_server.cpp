//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QUuid>
#include "cobotsys_background_json_server.h"

namespace cobotsys {

BackgroundJsonServer::BackgroundSlaveView::BackgroundSlaveView(
        std::function<void(const QJsonObject &, BackgroundSlaveView &)> jsonHandler){
    _decoder = std::make_shared<MessageDecoder>([=](const Message &m){ processMessage(m); });
    _json_handler = jsonHandler;
}


void BackgroundJsonServer::BackgroundSlaveView::processData(const QByteArray &ba){
    _decoder->decode(ba);
}

void BackgroundJsonServer::BackgroundSlaveView::processMessage(const distributed_system::Message &m){
    if (m.getType() == MessageType::Utf8BasedJSON) {
        QJsonParseError jsonParseError;
        QByteArray json(m.getContent(), m.getContentLength());
        auto jsonDoc = QJsonDocument::fromJson(json, &jsonParseError);
        if (jsonDoc.isNull()) {
            COBOT_LOG.warning() << "JSON: " << jsonParseError.errorString();
        } else {
            auto jsonObject = jsonDoc.object();
            if (_json_handler) {
                _json_handler(jsonObject, *this);
            }
        }
    }
}


double BackgroundJsonServer::CallbackTracker::calcTimeElapsed() const{
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - send_time;
    return diff.count();
}
//

}


namespace cobotsys {


BackgroundJsonServer::BackgroundJsonServer(QObject *parent) : ComputeNodeServer(parent){
    _instance_id = QUuid::createUuid().toString();
    _json_handler = [=](const QJsonObject &j){ COBOT_LOG.warning() << "Unhandled: " << j; };

    _chk_timer = new QTimer(this);
    _chk_timer->setInterval(100);
    connect(_chk_timer, &QTimer::timeout, this, &BackgroundJsonServer::timeoutChecker);
    _chk_timer->start();
}


BackgroundJsonServer::~BackgroundJsonServer(){
}

void BackgroundJsonServer::processClientData(QTcpSocket *clientLink, const QByteArray &ba){

    // 处理数据
    auto viewIter = _slaves.find(clientLink);
    if (viewIter != _slaves.end()) {
        viewIter->second->processData(ba);
    }
}


void BackgroundJsonServer::createClientView(QTcpSocket *clientLink){
    auto iter = _slaves.find(clientLink);
    if (iter == _slaves.end()) {
        auto view = std::make_shared<BackgroundSlaveView>([=](const QJsonObject &j, BackgroundSlaveView &v){
            processJson(j, clientLink, v);
        });

        _slaves.insert({clientLink, view});
    }
}

void BackgroundJsonServer::processJson(const QJsonObject &json,
                                   QTcpSocket *link,
                                   BackgroundJsonServer::BackgroundSlaveView &view){

    if (json.contains(JSON_REPLY) && json.contains(JSON_COMMAND_SEQ)) {
        auto seqn = json[JSON_COMMAND_SEQ].toString();

        auto iter = _jcmd_history.find(seqn);
        if (iter != _jcmd_history.end()) {
            JsonReply jrpy = {JsonReplyStatus::Success, json, iter->second.calcTimeElapsed()};
            if (iter->second.callback) {
                iter->second.callback(jrpy);
            }

            _jcmd_history.erase(iter);
        }
    } else {
        callJsonHandler(json);
    }
}


void BackgroundJsonServer::processClientConnect(QTcpSocket *tcpSocket){
    createClientView(tcpSocket);

    cmdGetName(tcpSocket);
}


void BackgroundJsonServer::processClientDisconnect(QTcpSocket *tcpSocket){
    auto iter = _slaves.find(tcpSocket);
    if (iter != _slaves.end()) {
        Q_EMIT clientDisconnected(iter->second->getName());
        _slaves.erase(tcpSocket);
    }
}

void BackgroundJsonServer::directWriteJson(QTcpSocket *clientLink, const QJsonObject &json){
    clientLink->write(MessageEncoder::genJsonMessage(json).getData());
}


void BackgroundJsonServer::cmdGetName(QTcpSocket *clientLink){
    QJsonObject json;
    json[JSON_COMMAND_KEY] = "GetSlaveName";

    writeJson(clientLink, json, [=](const JsonReply &rjs){
        if (rjs.reply_status == JsonReplyStatus::Success) {
            auto pView = _slaves[clientLink];
            pView->slave_instance_id = rjs.json_object[BACK_KEY_SLAVE_INSTANCE_ID].toString();
            pView->slave_name = rjs.json_object[BACK_KEY_SLAVE_NAME].toString();
            COBOT_LOG.info() << "Client Arrival: " << pView->slave_instance_id << ", " << pView->slave_name;

            Q_EMIT clientConnected(pView->getName());
        }
    });
}

void BackgroundJsonServer::writeJson(const QJsonObject &json,
                                 std::function<void(const cobotsys::JsonReply &)> on_slave_reply){
    if (json.contains(JSON_RECEIVER)) {
        auto receiver = json[JSON_RECEIVER].toString();
        for (auto &iter : _slaves) {
            if (iter.second->getName() == receiver) {
                writeJson(iter.first, json, on_slave_reply);
                return;
            }
        }
        COBOT_LOG.warning() << "No such target named: " << receiver;
    } else {
        for (auto &iter : _slaves) {
            writeJson(iter.first, json, on_slave_reply);
        }
    }
}

void BackgroundJsonServer::writeJson(QTcpSocket *clientLink,
                                 const QJsonObject &json,
                                 std::function<void(const cobotsys::JsonReply &)> on_slave_reply){
    auto localJson = json;
    auto seqNum = QUuid::createUuid().toString();

    localJson[JSON_COMMAND_SEQ] = QJsonValue(seqNum);

    CallbackTracker ctrack;
    ctrack.send_data = localJson;
    ctrack.send_time = std::chrono::high_resolution_clock::now();
    ctrack.callback = on_slave_reply;
    _jcmd_history.insert({seqNum, ctrack});
    directWriteJson(clientLink, localJson);
}

void BackgroundJsonServer::setJsonHandler(std::function<void(const QJsonObject &)> general_handler){
    _json_handler = general_handler;
}

void BackgroundJsonServer::callJsonHandler(const QJsonObject &json){
    if (_json_handler) {
        _json_handler(json);
    } else {
        COBOT_LOG.warning() << "Unhandled: " << json;
    }
}

void BackgroundJsonServer::timeoutChecker(){

    auto cur = std::chrono::high_resolution_clock::now();
    std::vector<QString> keys;
    for (auto &pair : _jcmd_history) {
        auto time_elapsed = pair.second.calcTimeElapsed();
        if (time_elapsed > 0.5) {
            if (pair.second.callback) {
                pair.second.callback({JsonReplyStatus::Success, pair.second.send_data, time_elapsed});
            }
            keys.push_back(pair.first);
        }
    }

    for (auto key : keys) {
        _jcmd_history.erase(key);
    }
}









//
}