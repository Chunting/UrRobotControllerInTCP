//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QJsonParseError>
#include "cobotsys_network_based_message_service.h"
#include "cobotsys_json_callback_manager.h"

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


public:
    MessageServerImpl(const CONFIG &conf, QObject *parent) : QObject(parent){
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
            sendMessage(MessageEncoder::genJsonMessage(j));
        });
    }


    void initBasicJsonHandler(){
        _json_callback_manager->addJsonCommandListener("GetMessageServiceServer", "reply", [=](const QJsonObject &json){
            COBOT_LOG.notice() << "Query Server: " << json;
            auto rjson = json;
            rjson[JSON_REPLY] = "";
            rjson["Server"] = "";
            sendMessage(MessageEncoder::genJsonMessage(rjson));
        });
    }

    void startAsService(){
    }

    void doQueryServiceServer(){
        QJsonObject json;
        json[JSON_COMMAND_KEY] = "GetMessageServiceServer";
        _json_callback_manager->writeJsonMessage(json, [=](const QJsonObject &j, JsonReplyStatus status){
            if (status == JsonReplyStatus::Timeout) {
                doQueryServiceServer();
            } else {
                COBOT_LOG.info() << "Success: " << j;
            }
        });
    }

    void startAsServer(){
        initBasicJsonHandler();

        doQueryServiceServer();
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
            _json_callback_manager->processJson(jsonDoc.object());
        }
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