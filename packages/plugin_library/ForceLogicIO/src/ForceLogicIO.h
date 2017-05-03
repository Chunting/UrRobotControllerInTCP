//
// Created by mingpeng on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef FORCELOGICIO_H
#define FORCELOGICIO_H

#include <QObject>
#include <cobotsys_abstract_controller.h>
//#include <../MotomanRobotDriver/src/MotomanRobotDriver.h>
using namespace cobotsys;

class ForceLogicIO : public QObject, public AbstractController
{   
Q_OBJECT
public:
    ForceLogicIO();
    bool setup(const QString& configFilePath);
    bool start();
    void stop();   
    void pause();
private:
    bool waitforLogicIOEnd(void);
    void onCallbackPosition(void);
    void unpack(void);
};

#endif // FORCELOGICIO_H
