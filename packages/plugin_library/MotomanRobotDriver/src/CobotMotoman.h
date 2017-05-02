//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_H
#define COBOT_MOTOMAN_H

#include <vector>
#include <stdint.h>
#include <cstdio>
#include <QByteArray>
#include <cobotsys.h>
//TODO FINISH

class CobotMotoman {
public:
    enum ROBOTCMD {
        CMD_START_UDP, CMD_SERVO_ON, CMD_SERVO_OFF, CMD_MOVE_ANGLE, CMD_MOVE_IMPULSE
    };
    static const int MAX_SOCKET_WAIT_ = 1000;
    static const size_t JOINT_NUM_ = 6;
    static const float MAX_ANGLE_INCREMENT_;
    static const int FRAME_LENGTH_=80;
    static const unsigned int TCP_PORT=11000;
    static const unsigned int UDP_PORT=11001;
};
using namespace cobotsys;
QByteArray IntToArray(qint32 source); //Use qint32 to ensure that the number have 4 bytes

#endif //COBOT_MOTOMAN_H
