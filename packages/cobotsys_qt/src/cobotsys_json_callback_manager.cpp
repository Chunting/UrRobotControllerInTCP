//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_json_callback_manager.h"
#include <chrono>
#include <QUuid>
#include <cobotsys_qt.h>

namespace cobotsys {


JsonCallbackManager::~JsonCallbackManager(){
}

void JsonCallbackManager::processJson(const QJsonObject &jsonObject){
    if (jsonObject.contains(JSON_COMMAND_SEQ) && jsonObject.contains(JSON_REPLY)) {
        if (jsonObject[JSON_RECEIVER].toString() == _json_receiver) {
            auto seqNum = jsonObject[JSON_COMMAND_SEQ].toString();
            auto iter = _json_write_callbacks.find(seqNum);
            if (iter != _json_write_callbacks.end()) {
                if (iter->second.callback) {
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> diff = end - iter->second.start;
                    iter->second.callback({JsonReplyStatus::Success, jsonObject, diff});
                }
                _json_write_callbacks.erase(seqNum);
            } else {
                COBOT_LOG.warning() << "Unknown Json Reply: " << jsonObject;
            }
        } else {
            // ignore json message if receiver is not same.
            //COBOT_LOG.info() << jsonObject;
        }
    } else if (jsonObject.contains(JSON_COMMAND_KEY)) {
        auto command = jsonObject[JSON_COMMAND_KEY].toString();

        auto iter = _json_command_listener_callbacks.find(command);
        if (iter != _json_command_listener_callbacks.end()) {
            for (auto &callPair : iter->second) {
                if (callPair.second) {
                    callPair.second(jsonObject);
                }
            }
        } else {
            COBOT_LOG.warning() << "Un-handled Command: " << jsonObject;
        }
    } else {
        COBOT_LOG.warning() << "Unknown-type JSON document: " << QJsonDocument(jsonObject).toJson().constData();
    }
}

bool JsonCallbackManager::addJsonCommandListener(const QString &jsonCommand, const QString &callbackName,
                                                 std::function<void(const QJsonObject &)> callback, bool alwaysAdd){

    auto &callList = _json_command_listener_callbacks[jsonCommand];
    auto call_iter = callList.find(callbackName);

    if (!alwaysAdd) {
        if (call_iter != callList.end()) {
            return false;
        }
    }

    callList[callbackName] = callback;
    return true;
}

JsonCallbackManager::JsonCallbackManager(const QString &receiverId){
    _json_receiver = receiverId;
}

QJsonObject JsonCallbackManager::writeJsonMessage(const QJsonObject &jsonObject,
                                                  std::function<void(const JsonReply &)> callback){

    auto localJson = jsonObject;
    auto seqNum = QUuid::createUuid().toString();

    localJson[JSON_COMMAND_SEQ] = QJsonValue(seqNum);
    auto &ctrack = _json_write_callbacks[seqNum];
    ctrack.sendTime = QDateTime::currentMSecsSinceEpoch();
    ctrack.callback = callback;
    ctrack.sendData = localJson;
    ctrack.start = std::chrono::high_resolution_clock::now();

    return localJson;
}

void JsonCallbackManager::checkTimeout(){
    auto cur_mtime = QDateTime::currentMSecsSinceEpoch();

    std::vector<QString> timeoutKeys;
    for (auto &pair : _json_write_callbacks) {
        auto diff_time = cur_mtime - pair.second.sendTime;
        if (diff_time > 500) {
            if (pair.second.callback) {
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> diff = end - pair.second.start;
                pair.second.callback({JsonReplyStatus::Timeout, pair.second.sendData, diff});
            } else {
                COBOT_LOG.notice() << "Timeout: " << pair.second.sendData << ", Diff: " << diff_time;
            }
            timeoutKeys.push_back(pair.first);
        }
    }

    for (auto &key : timeoutKeys) {
        _json_write_callbacks.erase(key);
    }
}
}