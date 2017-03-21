//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
#define PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_global_object_factory.h"
#include <QWidget>

namespace cobotsys {
class AbstractController : public AbstractObject {
public:
    AbstractController();
    virtual ~AbstractController();

    virtual bool start() = 0;
    virtual void pause() = 0;
    virtual void stop() = 0;

    virtual bool setup(const QString& configFilePath);
};
}


namespace cobotsys {
class AbstractControllerWidget : public QWidget, public AbstractController {
Q_OBJECT
public:
    AbstractControllerWidget();
    virtual ~AbstractControllerWidget();

    virtual bool setup(const QString& configFilePath);
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
