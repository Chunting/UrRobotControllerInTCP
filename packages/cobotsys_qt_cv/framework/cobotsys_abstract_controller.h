//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
#define PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H

#include "cobotsys_abstract_object.h"

namespace cobotsys {
class AbstractController : public AbstractObject {
public:
    AbstractController();
    virtual ~AbstractController();

    virtual bool start() = 0;
    virtual void pause() = 0;
    virtual void stop() = 0;

    virtual bool setup(const std::string& xmlConfigFilePath) = 0;
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_CONTROLLER_H
