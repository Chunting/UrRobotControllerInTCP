//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <cobotsys.h>
#include "JsonServer.h"


JsonServer::JsonServer(QObject* parent)
        : QObject(parent) {
    m_pWebSocket = nullptr;
    m_pWebSocketServer = new QWebSocketServer("CobotJsonSimulatork", QWebSocketServer::NonSecureMode, this);
    if (m_pWebSocketServer->listen(QHostAddress::Any, 45454)) {
        connect(m_pWebSocketServer, &QWebSocketServer::newConnection, this, &JsonServer::onNewConnection);
        COBOT_LOG.info("JsonServer") << "listening on port: " << 45454;
    }

    connect(this, &JsonServer::apiTaskUpdate, this, &JsonServer::replyTaskStage);

    m_acceptCmdKeys.push_back("START");
    m_acceptCmdKeys.push_back("STOP");
    m_acceptCmdKeys.push_back("PUSH_TASK");
    m_acceptCmdKeys.push_back("GET_REMAIN_TASK_NUM");
    m_acceptCmdKeys.push_back("CLEAR_TASK_QUEUE");

    m_curTask.id.clear();
    m_curTaskValid = false;
    m_isStarted = false;
    m_simulatorTimer = new QTimer(this);
    connect(m_simulatorTimer, &QTimer::timeout, this, &JsonServer::updateTaskStatus);
    m_simulatorTimer->setInterval(100);
    m_simulatorTimer->start();
}

JsonServer::~JsonServer() {
}

void JsonServer::onNewConnection() {
    if (m_pWebSocket) {
        QString errmsg = "Last connection still linked.";
        qDebug() << errmsg;

        QWebSocket* pSocket = m_pWebSocketServer->nextPendingConnection();
        pSocket->sendTextMessage(generateErrorReply("Last connection still linked."));
        pSocket->close();
        pSocket->deleteLater();
    } else {
        qDebug() << "Socket Connected.";
        m_pWebSocket = m_pWebSocketServer->nextPendingConnection();
        connect(m_pWebSocket, &QWebSocket::textMessageReceived, this, &JsonServer::processTextMessage);
        connect(m_pWebSocket, &QWebSocket::binaryMessageReceived, this, &JsonServer::processBinaryMessage);
        connect(m_pWebSocket, &QWebSocket::disconnected, this, &JsonServer::onSocketDisconnect);
    }
}

void JsonServer::processTextMessage(const QString& message) {
    QJsonParseError jsonError;
    auto jsonDoc = QJsonDocument::fromJson(message.toUtf8(), &jsonError);
    if (jsonError.error == QJsonParseError::NoError) {
        auto jsonObject = jsonDoc.object();
        if (checkSeqNum(jsonObject)) {
            if (checkCmdKey(jsonObject)) {
                dispatchJson(jsonObject);
            }
        }
    }
}

void JsonServer::processBinaryMessage(const QByteArray& message) {
}

void JsonServer::onSocketDisconnect() {
    COBOT_LOG.debug() << "Socket Closed.";
    m_pWebSocket->close();
    m_pWebSocket->deleteLater();
    m_pWebSocket = nullptr;
}

void JsonServer::replyJson(const QJsonObject& jsonObject) {
    if (m_pWebSocket) {
        auto str = QString::fromUtf8(QJsonDocument(jsonObject).toJson(QJsonDocument::Compact));
        m_pWebSocket->sendTextMessage(str);
        clearJsonSeq(jsonObject);
    }
}

void JsonServer::dispatchJson(const QJsonObject& jsonObject) {
    auto seqNum = jsonObject["SEQNUM"].toInt();
    auto cmdKey = jsonObject["COMMAND"].toString();

    m_cmdHistory[seqNum] = jsonObject;
    if (cmdKey == "START") onCmdStart(jsonObject);
    if (cmdKey == "STOP") onCmdStop(jsonObject);
    if (cmdKey == "PUSH_TASK") onCmdTask(jsonObject);
    if (cmdKey == "GET_REMAIN_TASK_NUM") onCmdGetRemain(jsonObject);
    if (cmdKey == "CLEAR_TASK_QUEUE") onCmdClear(jsonObject);
}

bool JsonServer::checkSeqNum(const QJsonObject& jsonObject) {
    if (jsonObject.contains("SEQNUM")) {
        int seqNum = jsonObject["SEQNUM"].toInt();
        auto iter = m_cmdHistory.find(seqNum);
        if (iter == m_cmdHistory.end()) {
            return true;
        }
        COBOT_LOG.debug() << "SEQNUM value already in used";
    }
    COBOT_LOG.debug() << "No SEQNUM key";
    return false;
}

bool JsonServer::checkCmdKey(const QJsonObject& jsonObject) {
    if (jsonObject.contains("COMMAND")) {
        auto cmdKey = jsonObject["COMMAND"].toString();
        if (isAcceptCmdKey(cmdKey)) {
            return true;
        }
        COBOT_LOG.debug() << "Unknown COMMAND: " << cmdKey;
    }
    COBOT_LOG.debug() << "No COMMAND key";
    return false;
}

bool JsonServer::isAcceptCmdKey(const QString& key) {
    for (const auto& iter : m_acceptCmdKeys) {
        if (iter == key) {
            return true;
        }
    }
    return false;
}

void JsonServer::onCmdStart(const QJsonObject& jsonObject) {
    auto rjson = jsonObject;
    rjson["COMMAND"] = jsonObject["COMMAND"].toString() + "_ACK";
    rjson["RESULT"] = "SUCCESS";
    if (m_isStarted) {
        rjson["RESULT_MESSAGE"] = "Already started!";
    }
    m_isStarted = true;
    replyJson(rjson);

    Q_EMIT reqStart();
}

void JsonServer::onCmdStop(const QJsonObject& jsonObject) {
    auto rjson = jsonObject;
    rjson["COMMAND"] = jsonObject["COMMAND"].toString() + "_ACK";
    rjson["RESULT"] = "SUCCESS";
    m_isStarted = false;
    replyJson(rjson);

    Q_EMIT reqStop();
}

void JsonServer::onCmdTask(const QJsonObject& jsonObject) {
    auto rjson = jsonObject;
    rjson["COMMAND"] = jsonObject["COMMAND"].toString() + "_ACK";
    if (m_isStarted) {
        QJsonArray taskList = jsonObject["TASK"].toArray();
        bool appendSuccess = true;
        for (const auto& iter : taskList) {
            auto obj = iter.toObject();
            if (!appendTask(obj)) {
                appendSuccess = false;
                break;
            }
        }

        if (appendSuccess) {
            rjson["RESULT"] = "SUCCESS";
        } else {
            rjson["RESULT"] = "FAILURE";
            rjson["RESULT_MESSAGE"] = "fail to append task.";
        }
    } else {
        rjson["RESULT"] = "FAILURE";
        rjson["RESULT_MESSAGE"] = "System is not started!";
    }
    replyJson(rjson);
}

void JsonServer::onCmdGetRemain(const QJsonObject& jsonObject) {
    auto rjson = jsonObject;
    rjson["COMMAND"] = jsonObject["COMMAND"].toString() + "_ACK";
    rjson["RESULT"] = "SUCCESS";
    rjson["REMAIN_TASK_NUM"] = (int) m_taskQueue.size();
    replyJson(rjson);
}

void JsonServer::onCmdClear(const QJsonObject& jsonObject) {
    auto rjson = jsonObject;
    rjson["COMMAND"] = jsonObject["COMMAND"].toString() + "_ACK";
    rjson["RESULT"] = "SUCCESS";
    m_taskQueue.clear();
    replyJson(rjson);
}

void JsonServer::clearJsonSeq(const QJsonObject& jsonObject) {
    auto seqNum = jsonObject["SEQNUM"].toInt();
    m_cmdHistory.erase(seqNum);
}

bool JsonServer::appendTask(const QJsonObject& jsonObject) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    TaskInfo taskInfo;
    taskInfo.id = jsonObject["ID"].toString();
    taskInfo.objFrom = jsonObject["OBJ_FROM"].toString();
    taskInfo.objTo = jsonObject["OBJ_TO"].toString();
    taskInfo.objInfo = jsonObject["OBJ_INFO"].toString();

    auto taskQueue = m_taskQueue;
    if (m_curTaskValid) {
        taskQueue.push_back(m_curTask);
    }
    for (auto& iter : taskQueue) {
        if (iter.id == taskInfo.id) {
            COBOT_LOG.debug() << "a new Task with same id that already in queue.";
            return false;
        }
    }
    m_taskQueue.push_back(taskInfo);
    return true;
}

void JsonServer::pickNextTask() {
    if (m_taskQueue.size()) {
        m_curTask = m_taskQueue.front();
        m_curTaskValid = true;
        m_curTaskStage = TaskStage::Begin;
        m_taskQueue.pop_front();
    } else {
        m_curTaskValid = false;
        m_curTask.id.clear();
        m_curTaskStage = TaskStage::End;
    }
}

void JsonServer::updateTaskStatus() {
    if (!m_isStarted) return;

    if (m_curTaskValid) {
        switch (m_curTaskStage) {
        case TaskStage::Begin: m_curTaskStage = TaskStage::PickFinish;
            break;
        case TaskStage::PickFinish:m_curTaskStage = TaskStage::DropFinish;
            break;
        case TaskStage::DropFinish:m_curTaskStage = TaskStage::End;
            break;
        case TaskStage::PickPlaceFailure:m_curTaskStage = TaskStage::PickPlaceFailure;
            break;
        case TaskStage::End: pickNextTask();
            break;
        }
    } else {
        pickNextTask();
    }
    reportCurStatus();
}

void JsonServer::reportCurStatus() {
    QJsonObject rjson;

    if (m_curTaskValid) {
        switch (m_curTaskStage) {
        case TaskStage::Begin:rjson["STATUS"] = "TASK_BEGIN";
            break;
        case TaskStage::PickFinish:rjson["STATUS"] = "TASK_PICK_FINISH";
            break;
        case TaskStage::DropFinish:rjson["STATUS"] = "TASK_DROP_FINISH";
            break;
        case TaskStage::PickPlaceFailure:rjson["STATUS"] = "TASK_FAIL";
            break;
        case TaskStage::End:rjson["STATUS"] = "TASK_END";
            break;
        }
        rjson["TASK_ID"] = m_curTask.id;
        replyJson(rjson);
    }
}

bool JsonServer::api_pickTask(JsonServer::TaskInfo& taskInfo) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    pickNextTask();
    taskInfo = m_curTask;
    return m_curTaskValid;
}

void JsonServer::api_setupTaskStage(const JsonServer::TaskInfo& taskInfo, JsonServer::TaskStage taskStage) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    Q_EMIT apiTaskUpdate(taskInfo.id, (int) taskStage);
}

void JsonServer::replyTaskStage(const QString& taskId, int taskStage) {
    QJsonObject rjson;
    rjson["TASK_ID"] = taskId;
    switch ((TaskStage) taskStage) {
    case TaskStage::Begin:rjson["STATUS"] = "TASK_BEGIN";
        break;
    case TaskStage::PickFinish:rjson["STATUS"] = "TASK_PICK_FINISH";
        break;
    case TaskStage::DropFinish:rjson["STATUS"] = "TASK_DROP_FINISH";
        break;
    case TaskStage::PickPlaceFailure:rjson["STATUS"] = "TASK_FAIL";
        break;
    case TaskStage::End:rjson["STATUS"] = "TASK_END";
        break;
    }
    if (rjson.contains("STATUS")) {
        COBOT_LOG.debug() << "Task :" << std::setw(8) << taskId << ", " << rjson["STATUS"].toString();
        replyJson(rjson);
    }
}

void JsonServer::api_debugTaskOnce(const QString& from_, const QString& to_) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    QJsonObject task;
    task["ID"] = "12345678";
    task["OBJ_FROM"] = from_;
    task["OBJ_TO"] = to_;
    task["OBJ_INFO"] = "box";

    appendTask(task);
}


QString generateErrorReply(const QString& errmsg) {
    QJsonObject rjson;
    rjson["ERROR"] = "ERROR";
    rjson["ERROR_MESSAGE"] = errmsg;
    return QString::fromUtf8(QJsonDocument(rjson).toJson());
}