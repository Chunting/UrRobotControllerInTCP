//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "EpsonRobotDriver.h"

#define DECL_ITEM(_type) { m_creator[( #_type )] = [](){ return std::make_shared<_type>(); }; }

class EpsonRobotDriverFactory : public cobotsys::AbstractObjectFactory {
public:
    std::map<std::string, std::function<AbstractObjectPtr()> > m_creator;

    EpsonRobotDriverFactory(){
        DECL_ITEM(EpsonRobotDriver)
    }

    virtual ~EpsonRobotDriverFactory(){
        INFO_DESTRUCTOR(this);
    }

    virtual std::vector<std::string> getSupportTypes(){
        std::vector<std::string> r;
        for (const auto& iter : m_creator)
            r.push_back(iter.first);
        return r;
    }

    virtual std::string getFactoryType(){
        return "EpsonRobotDriverFactory, Ver 1.0";
    }

    virtual std::shared_ptr<cobotsys::AbstractObject> createObject(const std::string& type){
        auto iter = m_creator.find(type);
        if (iter != m_creator.end()) {
            if (iter->second) {
                return iter->second();
            }
        }
        return std::shared_ptr<cobotsys::AbstractObject>();
    }
};

static std::shared_ptr<EpsonRobotDriverFactory> localFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (localFactory == nullptr) {
        localFactory = std::make_shared<EpsonRobotDriverFactory>();
    }

    return localFactory.get();
};