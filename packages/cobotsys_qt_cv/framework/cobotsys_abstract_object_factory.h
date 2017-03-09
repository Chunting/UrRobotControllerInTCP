//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H
#define PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H

#include <vector>
#include <string>
#include <memory>
#include <cobotsys_abstract_object.h>


namespace cobotsys {

class AbstractObjectFactory : public std::enable_shared_from_this<AbstractObjectFactory> {
public:
    AbstractObjectFactory();
    virtual ~AbstractObjectFactory();

    virtual std::vector<std::string> getSupportTypes() = 0;
    virtual std::string getFactoryType() = 0;
    virtual std::shared_ptr<AbstractObject> createObject(const std::string& type) = 0;
};


//
}

#endif //PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H
