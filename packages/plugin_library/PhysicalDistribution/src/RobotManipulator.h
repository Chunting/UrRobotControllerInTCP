//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_ROBOTMANIPULATOR_H
#define COBOTSYS_ROBOTMANIPULATOR_H

#include <cobotsys_abstract_widget.h>
#include "ui_RobotManipulator.h"

using namespace cobotsys;

class RobotManipulator : public AbstractWidget {
Q_OBJECT
public:
    RobotManipulator();
    virtual ~RobotManipulator();

    virtual bool setup(const QString& configFilePath);
    virtual void clearAttachedObject();
protected:
    Ui::RobotManipulator ui;
};


#endif //COBOTSYS_ROBOTMANIPULATOR_H
