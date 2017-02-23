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
#include <cobotsys_logger.h>

namespace cobotsys {

#define JSON_COMMAND_KEY    "Command"
#define JSON_COMMAND_SEQ    "SequenceNumber"
#define JSON_REPLY          "Reply"

enum class JsonReplyStatus {
    Success,
    Timeout,
};

class JsonCallbackManager {
public:
    JsonCallbackManager(std::function<void(const QJsonObject &)> jsonWriter);
    ~JsonCallbackManager();


    void processJson(const QJsonObject &jsonObject);

    bool addJsonCommandListener(const QString &jsonCommand, const QString &callbackName,
                                std::function<void(const QJsonObject &)> callback, bool alwaysAdd = true);


    void writeJsonMessage(const QJsonObject &jsonObject,
                          std::function<void(const QJsonObject &, JsonReplyStatus)> callback = nullptr);

    void checkTimeout();
protected:
    typedef std::map<QString, std::function<void(const QJsonObject &)> > JsonCommandListenerMap;
    std::map<QString, JsonCommandListenerMap> _json_command_listener_callbacks;

    std::function<void(const QJsonObject &)> _json_writer;

    struct JsonCallbackTracker {
        qint64 sendTime;
        QJsonObject sendData;
        std::function<void(const QJsonObject &, JsonReplyStatus)> callback;
    };

    std::map<QString, JsonCallbackTracker> _json_write_callbacks;
};
}


#endif //PROJECT_COBOTSYS_JSON_CALLBACK_MANAGER_H
