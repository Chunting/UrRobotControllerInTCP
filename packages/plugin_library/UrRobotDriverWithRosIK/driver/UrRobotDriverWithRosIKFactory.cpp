//
// Created by 潘绪洋 on 17-3-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include <ros/ros.h>
#include "UrAdapterWithIK.h"

class UrRobotDriverWithRosIKFactory : public cobotsys::AbstractObjectFactory {
public:
    UrRobotDriverWithRosIKFactory(){
    }

    virtual ~UrRobotDriverWithRosIKFactory(){
        INFO_DESTRUCTOR(this);
    }

    virtual std::vector<std::string> getSupportTypes(){
        return {"UrIK"};
    }

    virtual std::string getFactoryType(){
        return "UrRobotDriverWithRosIKFactory, Ver 1.0";
    }

    virtual std::shared_ptr<cobotsys::AbstractObject> createObject(const std::string& type){
        if (type == "UrIK")
            return std::make_shared<UrAdapterWithIK>();
        return std::shared_ptr<cobotsys::AbstractObject>();
    }
};

std::shared_ptr<UrRobotDriverWithRosIKFactory> localFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (localFactory == nullptr) {
        localFactory = std::make_shared<UrRobotDriverWithRosIKFactory>();
    }

    return localFactory.get();
};