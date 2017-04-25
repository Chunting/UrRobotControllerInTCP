//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_JSONSERVER_H
#define COBOTSYS_JSONSERVER_H

#include <QObject>
#include <QWebSocketServer>
#include <QWebSocket>
#include <QDebug>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <map>
#include <deque>
#include <QTimer>


QString generateErrorReply(const QString& errmsg);

class JsonServer : public QObject {
Q_OBJECT
public:
    struct TaskInfo {
        QString id;
        QString objFrom;
        QString objTo;
        QString objInfo;
    };

    enum class TaskStage {
        Begin,
        PickFinish,
        DropFinish,
        End,
    };
public:
    JsonServer(QObject* parent = nullptr);
    virtual ~JsonServer();

    bool api_pickTask(TaskInfo& taskInfo);
    void api_setupTaskStage(const TaskInfo& taskInfo, TaskStage taskStage);

Q_SIGNALS:
    void reqStart();
    void reqStop();
    void reqTask();

protected:
    void onNewConnection();

    void processTextMessage(const QString& message);
    void processBinaryMessage(const QByteArray& message);
    void onSocketDisconnect();

    void replyJson(const QJsonObject& jsonObject);
    void dispatchJson(const QJsonObject& jsonObject);

    bool checkSeqNum(const QJsonObject& jsonObject);
    bool checkCmdKey(const QJsonObject& jsonObject);

    bool isAcceptCmdKey(const QString& key);

protected:
    void onCmdStart(const QJsonObject& jsonObject);
    void onCmdStop(const QJsonObject& jsonObject);
    void onCmdTask(const QJsonObject& jsonObject);
    void onCmdGetRemain(const QJsonObject& jsonObject);
    void onCmdClear(const QJsonObject& jsonObject);

    void clearJsonSeq(const QJsonObject& jsonObject);


    bool appendTask(const QJsonObject& jsonObject);

    void updateTaskStatus();

    void reportCurStatus();
    void pickNextTask();
protected:
    QWebSocketServer* m_pWebSocketServer;
    QWebSocket* m_pWebSocket;

    bool m_isStarted;

    std::map<int, QJsonObject> m_cmdHistory;

    std::vector<QString> m_acceptCmdKeys;



    std::deque<TaskInfo> m_taskQueue;
    QTimer* m_simulatorTimer;



    TaskInfo m_curTask;
    bool m_curTaskValid;
    TaskStage m_curTaskStage;
};


#endif //COBOTSYS_JSONSERVER_H
