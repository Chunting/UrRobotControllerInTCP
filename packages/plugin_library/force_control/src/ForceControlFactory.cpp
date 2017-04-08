//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include <src/emptyWidget.h>

#define DECL_ITEM(_type) { m_creator[( #_type )] = [](){ return std::make_shared<_type>(); }; }

class ForceControlFactory : public cobotsys::AbstractObjectFactory {
public:
    std::map<std::string, std::function<AbstractObjectPtr()> > m_creator;

    ForceControlFactory(){
       DECL_ITEM(emptyWidget);

    }

    virtual ~ForceControlFactory(){
        INFO_DESTRUCTOR(this);
    }

    virtual std::vector<std::string> getSupportTypes(){
        std::vector<std::string> r;
        for (const auto& iter : m_creator)
            r.push_back(iter.first);
        return r;
    }

    virtual std::string getFactoryType(){
        return "ForceControlFactory, Ver 1.0";
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

static std::shared_ptr<ForceControlFactory> localFactory;

extern "C" void* getAbstractObjectFactoryInstance(){
    if (localFactory == nullptr) {
        localFactory = std::make_shared<ForceControlFactory>();
    }

    return localFactory.get();
};