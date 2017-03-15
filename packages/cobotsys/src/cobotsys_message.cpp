//
// Created by 潘绪洋 on 17-2-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_message.h"

namespace cobotsys {
namespace distributed_system {

#define MESSAGE_HEADER_MAGIC_NUM 0x12345656
#define MESSAGE_CHECKSUM_SIZE   sizeof(uint16_t)

MessageEncoder::MessageEncoder(){
}

MessageEncoder::~MessageEncoder(){
}

Message MessageEncoder::genStringMessage(const std::string &msg){
    return genMessage(QByteArray(msg.c_str(), msg.size() + 1), MessageType::String);
}


Message MessageEncoder::genStringMessage(const QString &msg){
    return genStringMessage(std::string(msg.toLocal8Bit().constData()));
}

Message MessageEncoder::genJsonMessage(const QJsonObject &msg){
    return genMessage(QJsonDocument(msg).toJson(), MessageType::Utf8BasedJSON);
}

Message MessageEncoder::genMessage(const QByteArray &msg, MessageType type){
    QByteArray ba;
    MessageHeader msg_hdr;

    // build header
    msg_hdr.magic_num = MESSAGE_HEADER_MAGIC_NUM;
    msg_hdr.length = msg.size() + sizeof(msg_hdr) + MESSAGE_CHECKSUM_SIZE;
    msg_hdr.type = (uint32_t) type;

    // push head and body
    ba.append((const char *) &msg_hdr, sizeof(msg_hdr));
    ba.append(msg);

    // calc checksum
    uint16_t checksum = qChecksum(ba.constData(), ba.size());

    // push checksum
    ba.append((const char *) &checksum, sizeof(checksum));

    // final message
    return Message(ba);
}


Message::Message(const Message &r) : _data(r._data){
}

Message::Message(const QByteArray &ba) : _data(ba){
}

Message::Message(){
}

Message::~Message(){
}

bool Message::isValid() const{
    if (_data.size() >= MESSAGE_CHECKSUM_SIZE) {
        auto real_checksum = qChecksum(_data.constData(), _data.size() - MESSAGE_CHECKSUM_SIZE);
        auto msgr_checksum = *(uint16_t *) (_data.constData() + _data.size() - MESSAGE_CHECKSUM_SIZE);
        return (real_checksum == msgr_checksum);
    }
    return false;
}

const QByteArray &Message::getData() const{
    return _data;
}


const char *Message::getContent() const{
    return (_data.constData() + sizeof(MessageHeader));
}

MessageType Message::getType() const{
    return (MessageType) ((MessageHeader *) _data.constData())->type;
}

int Message::getContentLength() const{
    int rsize = _data.size() - (int) sizeof(MessageHeader) - MESSAGE_CHECKSUM_SIZE;
    if (rsize >= 0)
        return rsize;
    return 0;
}

bool MessageDecoder::extractMessage(int pos_magic){
    auto msg_hdr = (const MessageHeader *) (_prev_data.constData() + pos_magic);

    int remain_size = _prev_data.size() - pos_magic;
    int msg_length = (int) msg_hdr->length;

    if (msg_length <= remain_size) {
        auto msg_ba = _prev_data.mid(pos_magic, msg_length);
        if (_handler) {
            Message msg_obj(msg_ba);
            if (msg_obj.isValid())
                _handler(msg_obj);
            else
                COBOT_LOG.warning() << "MessageDecoder: " << "Recv Message with Wrong Checksum.";
        }

        _prev_data = _prev_data.mid(pos_magic + msg_length);
        return true;
    }
    return false;
}

void MessageDecoder::decode(const QByteArray &ba){
    cacheData(ba);

    int pos_magic = 0;

    while (_prev_data.size()) {
        pos_magic = _prev_data.indexOf(_ba_magic);
        if (pos_magic < 0)
            return; // 在已知数据里没有发现数据包标识

        if (!extractMessage(pos_magic))
            return;
    }
}

MessageDecoder::MessageDecoder(std::function<void(const Message &)> msg_handler){
    _handler = msg_handler;

    uint32_t magic_num = MESSAGE_HEADER_MAGIC_NUM;
    _ba_magic.append((const char *) &magic_num, sizeof(magic_num));
}

MessageDecoder::~MessageDecoder(){
}

void MessageDecoder::cacheData(const QByteArray &ba){
    if (_prev_data.size())
        _prev_data.append(ba);
    else
        _prev_data = ba;

    if (_prev_data.size() > 1024 * 1024 * 16) {
        COBOT_LOG.warning() << "Seem no message was found.";
        _prev_data.clear();
    }
}
}
}