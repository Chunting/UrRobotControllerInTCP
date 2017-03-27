//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ARMROBOTMANIPULATOR_H
#define PROJECT_ARMROBOTMANIPULATOR_H

#include <cobotsys_abstract_widget.h>

using namespace cobotsys;

class ArmRobotManipulator : public AbstractWidget {
    Q_OBJECT
public:
    ArmRobotManipulator();
    virtual ~ArmRobotManipulator();

    virtual bool setup(const QString& configFilePath);
};


#endif //PROJECT_ARMROBOTMANIPULATOR_H
