//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "UrRobotDriverFactory.h"
#include "UrAdapter.h"

std::shared_ptr<UrRobotDriverFactory> localFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (localFactory == nullptr) {
        localFactory = std::make_shared<UrRobotDriverFactory>();
    }

    return localFactory.get();
};

UrRobotDriverFactory::UrRobotDriverFactory(){
}

UrRobotDriverFactory::~UrRobotDriverFactory(){
    INFO_DESTRUCTOR(this);
}

std::vector<std::string> UrRobotDriverFactory::getSupportTypes(){
    return {"UrAdapter"};
}

std::string UrRobotDriverFactory::getFactoryType(){
    return "UrRobotDriverFactory, Ver 1.0";
}

std::shared_ptr<cobotsys::AbstractObject> UrRobotDriverFactory::createObject(const std::string& type){
    if (type == "UrAdapter")
        return std::make_shared<UrAdapter>();
    return std::shared_ptr<cobotsys::AbstractObject>();
}
