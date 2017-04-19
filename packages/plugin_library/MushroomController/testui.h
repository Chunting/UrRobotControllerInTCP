//
// Created by zhangshaohua on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_TESTUI_H
#define COBOTSYS_TESTUI_H

#include <cobotsys_abstract_widget.h>

using namespace cobotsys;

class testui : public AbstractWidget {
    Q_OBJECT
public:
    testui();
    ~testui();

    virtual bool setup(const QString& configFilePath);
};

#endif //COBOTSYS_TESTUI_H
