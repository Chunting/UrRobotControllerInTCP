//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_SIMPLECONTROLLERFACTORY_H
#define PROJECT_SIMPLECONTROLLERFACTORY_H

#include <cobotsys_abstract_object_factory.h>

class SimpleControllerFactory : public cobotsys::AbstractObjectFactory {
public:
    SimpleControllerFactory();
    virtual ~SimpleControllerFactory();

    virtual std::vector<std::string> getSupportTypes();
    virtual std::string getFactoryType();
    virtual std::shared_ptr<cobotsys::AbstractObject> createObject(const std::string& type);
};


#endif //PROJECT_SIMPLECONTROLLERFACTORY_H
