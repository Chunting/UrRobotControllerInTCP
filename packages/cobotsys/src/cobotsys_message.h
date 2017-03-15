//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_MESSAGE_H
#define PROJECT_COBOTSYS_MESSAGE_H

#include <QByteArray>
#include <QString>
#include <QJsonObject>
#include <QJsonDocument>
#include <functional>
#include <memory>
#include <cobotsys_logger.h>

namespace cobotsys {
namespace distributed_system {

enum class MessageType {
    String,
    Utf8BasedXML,
    Utf8BasedJSON,
    StructuredData,
};

struct MessageHeader {
    uint32_t magic_num;
    uint32_t length;
    uint32_t type;
};

class Message;
class MessageDispatcher;
class MessageEncoder;
class MessageDecoder;

class Message {
public:
    Message(const Message &r);
    Message(const QByteArray &ba);
    Message();
    ~Message();

    MessageType getType() const;
    const char *getContent() const;
    int getContentLength() const;

    bool isValid() const;

    const QByteArray &getData() const;
protected:
    QByteArray _data; // raw
};


class MessageEncoder {
public:
    MessageEncoder();
    ~MessageEncoder();


    static Message genStringMessage(const QString &msg);
    static Message genStringMessage(const std::string &msg);
    static Message genJsonMessage(const QJsonObject &msg);

    static Message genMessage(const QByteArray &msg, MessageType type);
protected:
    QByteArray ba;
};


class MessageDecoder {
public:
    MessageDecoder(std::function<void(const Message &msg)> msg_handler);
    ~MessageDecoder();

    void decode(const QByteArray &ba);

protected:
    void cacheData(const QByteArray &ba);
    bool extractMessage(int pos_magic);
protected:
    std::function<void(const Message &msg)> _handler;
    QByteArray _prev_data;
    QByteArray _ba_magic;
};
}
}


#endif //PROJECT_COBOTSYS_MESSAGE_H
