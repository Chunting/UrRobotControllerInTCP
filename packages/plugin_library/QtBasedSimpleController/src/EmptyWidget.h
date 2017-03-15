//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EMPTYWIDGET_H
#define PROJECT_EMPTYWIDGET_H


#include <cobotsys_abstract_controller.h>

class EmptyWidget : public cobotsys::AbstractControllerWidget {
Q_OBJECT
public:
    EmptyWidget();
    virtual ~EmptyWidget();

    virtual bool start();
    virtual void pause();
    virtual void stop();
};


#endif //PROJECT_EMPTYWIDGET_H
