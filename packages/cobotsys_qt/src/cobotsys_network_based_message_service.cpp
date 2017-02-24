//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QJsonParseError>
#include <QtCore/QUuid>
#include "cobotsys_network_based_message_service.h"
#include "cobotsys_json_callback_manager.h"


QHostAddress localIPv4(){
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for (auto &address : list) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
            return address;
    }
    return QHostAddress(QHostAddress::LocalHost);
}


namespace cobotsys {

namespace distributed_system {

class MessageService::MessageServerImpl : public QObject {
public:
    QUdpSocket *_udp_socket;
    QTimer *_timeout_timer;
    uint32_t _udp_message_no;
    CONFIG _config;

    std::shared_ptr<MessageDecoder> _decoder;
    std::function<void(const Message &msg)> _message_handler;

    std::shared_ptr<JsonCallbackManager> _json_callback_manager;

    QString _server_id;
    QString _instance_id;
    QString _instance_id_short;

public:
    MessageServerImpl(const CONFIG &conf, QObject *parent) : QObject(parent){
        _instance_id = QUuid::createUuid().toString();
        _instance_id_short = _instance_id.mid(1, 8);
        _config = conf;
        _udp_message_no = 0;

        _udp_socket = new QUdpSocket(this);
        _udp_socket->bind(QHostAddress::AnyIPv4, conf.port, QUdpSocket::ShareAddress);
        _udp_socket->joinMulticastGroup(conf.groupAddress);
        _udp_socket->setSocketOption(QAbstractSocket::MulticastTtlOption, conf.ttl);
        connect(_udp_socket, &QUdpSocket::readyRead, this, &MessageServerImpl::processPendingDatagrams);

        _timeout_timer = new QTimer(this);
        _timeout_timer->setInterval(100);
        connect(_timeout_timer, &QTimer::timeout, this, &MessageServerImpl::timeoutTickUpdate);
        _timeout_timer->start();

        _message_handler = [=](const Message &msg){ messageHandler(msg); };
        _decoder = std::shared_ptr<MessageDecoder>(new MessageDecoder(_message_handler));

        _json_callback_manager = std::make_shared<JsonCallbackManager>([=](const QJsonObject &j){
            sendJsonMessage(j);
        }, _instance_id);

        COBOT_LOG.notice() << "Instance ID: " << _instance_id;
    }

    uint16_t getServiceServerPort() const{
        return _config.port + 1;
    }

    void initServerJsonHandler(){
        _json_callback_manager->addJsonCommandListener("GetMessageServiceServer", "reply", [=](const QJsonObject &json){
            COBOT_LOG.info() << "Query : " << json;
            auto rjson = json;
            rjson[JSON_REPLY] = "";
            rjson["Server"] = _server_id;
            rjson["ServerIp"] = localIPv4().toString();
            rjson["ServerPort"] = getServiceServerPort();
            rjson[JSON_RECEIVER] = rjson[JSON_SENDER].toString();
            rjson.remove(JSON_SENDER);
            rjson.remove(JSON_COMMAND_KEY);
            COBOT_LOG.info() << "Reply : " << rjson;
            sendJsonMessage(rjson);
        });
    }

    void startAsService(){
    }

    void queryIfServerExist(std::function<void()> on_no_server){
        QJsonObject json;
        json[JSON_COMMAND_KEY] = "GetMessageServiceServer";
        json[JSON_SENDER] = _instance_id;
        _json_callback_manager->writeJsonMessage(json, [=](const JsonReply &reply){
            if (reply.replyStatus == JsonReplyStatus::Timeout) {
                if (on_no_server)
                    on_no_server();
            } else {
                COBOT_LOG.info() << "Reply : " << reply.jsonObject << ", " << reply.timeUsed.count() * 1000 << "ms";
            }
        });
    }

    void doQueryServiceServer(){
        QJsonObject json;
        json[JSON_COMMAND_KEY] = "GetMessageServiceServer";
        json[JSON_SENDER] = _instance_id;
        _json_callback_manager->writeJsonMessage(json, [=](const JsonReply &reply){
            if (reply.replyStatus == JsonReplyStatus::Timeout) {
                doQueryServiceServer();
            } else {
                COBOT_LOG.info() << "Reply : " << reply.jsonObject << ", " << reply.timeUsed.count() * 1000 << "ms";
            }
        });
    }

    void genServerId(){
        std::stringstream oss;
        oss << _instance_id << " - PID {"
            << QCoreApplication::applicationPid() << "}";
        _server_id = QString::fromLocal8Bit(oss.str().c_str());
    }

    void startAsServer(){
        queryIfServerExist([=](){
            genServerId();
            initServerJsonHandler();
            doQueryServiceServer();
        });
    }

    void messageHandler(const Message &msg){
        if (msg.getType() == MessageType::String) { COBOT_LOG.info() << msg.getContent(); }
        if (msg.getType() == MessageType::Utf8BasedJSON) jsonHandler(msg);

        timeoutTickUpdate();
    }

    void jsonHandler(const Message &msg){
        QJsonParseError jsonParseError;
        QByteArray json(msg.getContent(), msg.getContentLength());
        auto jsonDoc = QJsonDocument::fromJson(json, &jsonParseError);
        if (jsonDoc.isNull()) {
            COBOT_LOG.warning() << "JSON: " << jsonParseError.errorString();
        } else {
            auto jsonObject = jsonDoc.object();
            if (jsonObject.contains(JSON_COMMAND_KEY)) {
                if (_server_id.size()) {
                    _json_callback_manager->processJson(jsonDoc.object());
                }
            } else {
                _json_callback_manager->processJson(jsonDoc.object());
            }
        }
    }

    void sendJsonMessage(const QJsonObject &json){
        sendMessage(MessageEncoder::genJsonMessage(json));
    }

    void sendMessage(const Message &msg){
        _udp_socket->writeDatagram(msg.getData(), _config.groupAddress, _config.port);
    }

    void sendDebugMessage(){
        std::stringstream oss;
        oss << "PID: " << std::setw(6) << QCoreApplication::applicationPid() << ", Num: " << _udp_message_no++;
        auto msg = MessageEncoder::genStringMessage(oss.str());

        sendMessage(msg);
    }

    void timeoutTickUpdate(){
        _json_callback_manager->checkTimeout();
    }

    void processPendingDatagrams(){
        QByteArray datagram;
        while (_udp_socket->hasPendingDatagrams()) {
            datagram.resize(_udp_socket->pendingDatagramSize());
            auto size_readed = _udp_socket->readDatagram(datagram.data(), datagram.size());
            if (size_readed) {
                _decoder->decode(QByteArray(datagram.constData(), size_readed));
            }
        }
    }
};

MessageService::CONFIG::CONFIG(){
    groupAddress = QHostAddress("239.255.43.21");
    port = 45455;
    ttl = 2;
}
}
}


namespace cobotsys {

namespace distributed_system {
static std::shared_ptr<MessageService> message_service;

MessageService::MessageService(const MessageService::CONFIG &conf, QObject *parent) : QObject(parent){
    _priv = new MessageServerImpl(conf, this);
}


MessageService::~MessageService(){
}

void MessageService::lanuchService(const MessageService::CONFIG &conf){
    if (message_service == nullptr) {
        message_service = std::shared_ptr<MessageService>(new MessageService(conf));
        message_service->_priv->startAsService();
    }
}


void MessageService::lanuchServiceServer(const MessageService::CONFIG &conf){

    if (message_service == nullptr) {
        message_service = std::shared_ptr<MessageService>(new MessageService(conf));
        message_service->_priv->startAsServer();
    }
}

void MessageService::send(const QString &target, const QByteArray &data){
}
}
}