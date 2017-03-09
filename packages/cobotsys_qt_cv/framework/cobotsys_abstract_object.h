//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_OBJECT_H
#define PROJECT_COBOTSYS_ABSTRACT_OBJECT_H

#include <memory>

namespace cobotsys {


class AbstractObject : public std::enable_shared_from_this<AbstractObject> {
public:
    AbstractObject();
    virtual ~ AbstractObject();
};


//
}


#endif //PROJECT_COBOTSYS_ABSTRACT_OBJECT_H
