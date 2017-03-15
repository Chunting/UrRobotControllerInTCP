//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URROBOTDRIVERFACTORY_H
#define PROJECT_URROBOTDRIVERFACTORY_H


#include <cobotsys_abstract_object_factory.h>

class UrRobotDriverFactory : public cobotsys::AbstractObjectFactory {
public:
    UrRobotDriverFactory();
    virtual ~UrRobotDriverFactory();

    virtual std::vector<std::string> getSupportTypes();
    virtual std::string getFactoryType();
    virtual std::shared_ptr<cobotsys::AbstractObject> createObject(const std::string& type);
};

#endif //PROJECT_URROBOTDRIVERFACTORY_H
