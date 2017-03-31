//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
#define PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_global_object_factory.h"
#include "cobotsys_abstract_widget.h"
#include <QWidget>

namespace cobotsys {
/**
 * AbstractController
 * 用于流程控制的外部接口，从用户界面的角度来说
 * 适用于从单一工作点移动到另一个单一工作点。
 *
 * 对应于界面上的三个按钮
 */
class AbstractController : public AbstractObject {
public:
    AbstractController();
    virtual ~AbstractController();

    /**
     *
     * @return
     */
    virtual bool start() = 0;
    virtual void pause() = 0;
    virtual void stop() = 0;
};
}


namespace cobotsys {
class AbstractControllerWidget : public QWidget, public AbstractController {
public:
    AbstractControllerWidget();
    virtual ~AbstractControllerWidget();
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
