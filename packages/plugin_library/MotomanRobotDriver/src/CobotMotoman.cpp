//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CobotMotoman.h"
const float CobotMotoman::MAX_ANGLE_INCREMENT_=1.0;
using namespace cobotsys;
QByteArray IntToArray(qint32 source) //Use qint32 to ensure that the number have 4 bytes
{
    //Avoid use of cast, this is the Qt way to serialize objects
    QByteArray temp;
    QDataStream data(&temp, QIODevice::ReadWrite);
    data << source;
    return temp;
}