//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_JSON_CALLBACK_MANAGER_H
#define PROJECT_COBOTSYS_JSON_CALLBACK_MANAGER_H

#include <QJsonObject>
#include <QJsonDocument>
#include <QDateTime>
#include <QTimer>
#include <functional>
#include <map>
#include <chrono>
#include <cobotsys_logger.h>

namespace cobotsys {

#define JSON_COMMAND_KEY    "Command"
#define JSON_COMMAND_SEQ    "SequenceNumber"
#define JSON_REPLY          "Reply"
#define JSON_SENDER         "Sender"
#define JSON_RECEIVER       "Receiver"

enum class JsonReplyStatus {
    Success,
    Timeout,
};

struct JsonReply {
    JsonReplyStatus replyStatus;
    const QJsonObject &jsonObject;
    std::chrono::duration<double> timeUsed; // Second
};


class JsonCallbackManager {
public:
    JsonCallbackManager(std::function<void(const QJsonObject &)> jsonWriter, const QString &receiverId);
    ~JsonCallbackManager();


    void processJson(const QJsonObject &jsonObject);

    bool addJsonCommandListener(const QString &jsonCommand, const QString &callbackName,
                                std::function<void(const QJsonObject &)> callback, bool alwaysAdd = true);


    void writeJsonMessage(const QJsonObject &jsonObject,
                          std::function<void(const JsonReply &reply)> callback = nullptr);

    void checkTimeout();
protected:
    typedef std::map<QString, std::function<void(const QJsonObject &)> > JsonCommandListenerMap;
    std::map<QString, JsonCommandListenerMap> _json_command_listener_callbacks;

    std::function<void(const QJsonObject &)> _json_writer;

    struct JsonCallbackTracker {
        qint64 sendTime;
        QJsonObject sendData;
        std::function<void(const JsonReply &reply)> callback;
        std::chrono::high_resolution_clock::time_point start;
    };

    std::map<QString, JsonCallbackTracker> _json_write_callbacks;
    QString _json_receiver;
};
}


#endif //PROJECT_COBOTSYS_JSON_CALLBACK_MANAGER_H
