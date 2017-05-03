//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CobotMotoman.h"

using namespace cobotsys;
QByteArray IntToArray(qint32 source) //Use qint32 to ensure that the number have 4 bytes
{
    //Avoid use of cast, this is the Qt way to serialize objects
    QByteArray temp;
    QDataStream data(&temp, QIODevice::ReadWrite);
    data << source;
    return temp;
}
MotomanRobotState::MotomanRobotState(std::condition_variable& msg_cond) {
    q_target_.resize(JOINT_NUM,0.0); //Target joint positions
    q_actual_.resize(JOINT_NUM,0.0); //Actual joint positions
    digital_input_bits_.resize(DIGITAL_INPUT_NUM,false); //Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
    pos_actual_.resize(6,0.0);;//Actual cartesian position.
    pMsg_cond_ = &msg_cond;
}
void MotomanRobotState::unpack(QByteArray &msg) {
    const int RECV_FRAME_LENGTH_=82;
    if(msg.size()==RECV_FRAME_LENGTH_ && (quint8)msg[0]==0xf0 && (quint8)msg[RECV_FRAME_LENGTH_-1]==0xf0){
        for(int i=0;i<6;i++){
            q_actual_[i]=(double)(msg.mid(49+i*4,4).toLong())/FLOAT_PRECISION;
        }
        for(int i=0;i<3;i++){
            pos_actual_[i]=(double)(msg.mid(1+i*4,4).toLong())*0.001;
            pos_actual_[i+3]=(double)(msg.mid(13+i*4,4).toLong())/FLOAT_PRECISION;
        }
        for(int i=0;i<8;i++){
            digital_input_bits_[i]=((quint8)msg[73+i]!=0);
        }
    }else{
        COBOT_LOG.error()<<"Received frame format error.";
    }
}

void MotomanRobotState::setVesion() {
    major_version_=1;
    minor_version_=0;
}

std::string MotomanRobotState::getVersion() {
    return "1.0";
}

std::vector<double> MotomanRobotState::getQActual() {
    return q_actual_;
}

int MotomanRobotState::getDigitalOutputBits() {
    COBOT_LOG.warning()<<"Get digital output bits unrealized yet.";
    return 0;
}
int MotomanRobotState::getDigitalInputBits() {
    int result=0;
    for(int i=0;i<8;i++){
        result+=result*2+(int)digital_input_bits_[i];
    }
    return 0;
}


