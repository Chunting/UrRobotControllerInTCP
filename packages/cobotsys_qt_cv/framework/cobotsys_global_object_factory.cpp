//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_global_object_factory.h"

#define OBJECT_FACTORY_SYMBOL "getAbstractObjectFactoryInstance"

typedef void* (* getAbstractObjectFactoryInstance)();

namespace cobotsys {
static GlobalObjectFactory* g_defaultObjectFactory = nullptr;

class GlobalObjectFactory::GlobalObjectFactoryImpl {
public:
    std::map<std::string, std::shared_ptr<AbstractObjectFactory> > objectFactoryMap;

    bool hasFactory(const std::string& factoryType) const{
        return (objectFactoryMap.find(factoryType) != objectFactoryMap.end());
    }

    bool hasFactory(std::shared_ptr<AbstractObjectFactory>& factory){
        return hasFactory(factory->getFactoryType());
    }

    void appendFactory(std::shared_ptr<AbstractObjectFactory>& factory){
        objectFactoryMap[factory->getFactoryType()] = factory;
    }


    void loadLibrary(const QFileInfo& fileInfo){
        auto fAddr = QLibrary::resolve(fileInfo.absoluteFilePath(), OBJECT_FACTORY_SYMBOL);
        auto qFunc = (getAbstractObjectFactoryInstance) fAddr;

        if (qFunc) {
            auto rawFactory = static_cast<AbstractObjectFactory*>(qFunc());
            auto shrFactory = rawFactory->shared_from_this();
            auto factoryType = shrFactory->getFactoryType();

            if (hasFactory(shrFactory)) {
            } else {
                appendFactory(shrFactory);
                auto tlist = shrFactory->getSupportTypes();

                std::stringstream oss;
                oss << "{";
                for (size_t i = 0; i < tlist.size(); i++) {
                    oss << tlist[i];
                    if (i + 1 < tlist.size()) oss << ", ";
                }
                oss << "}";


                COBOT_LOG.message("Plugin")
                        << std::setw(32) << fileInfo.fileName() << ": "
                        << shrFactory->getFactoryType()
                        << ", " << oss.str();
            }
        }
    }

    std::shared_ptr<AbstractObject> createObject(const std::string& factory, const std::string& type){
        auto iter = objectFactoryMap.find(factory);
        if (iter != objectFactoryMap.end()) {
            auto result = iter->second->createObject(type);
            if (result) {
                COBOT_LOG.message("Cr-Object") << factory << ", " << type;
            }
            return result;
        }
        return nullptr;
    }
};
}


namespace cobotsys {

GlobalObjectFactory::GlobalObjectFactory()
        : m_impl(new GlobalObjectFactoryImpl){
    g_defaultObjectFactory = this;
}

GlobalObjectFactory::~GlobalObjectFactory(){
}

std::shared_ptr<AbstractObject> GlobalObjectFactory::createObject(const std::string& factory, const std::string& type){
    return m_impl->createObject(factory, type);
}

void GlobalObjectFactory::loadLibrarys(const QString& path){
    QDir dirPath(path);
    if (dirPath.exists()) {
        auto fileInfoList = dirPath.entryInfoList(QDir::Files);

        for (auto fileInfo : fileInfoList) {
            if (QLibrary::isLibrary(fileInfo.absoluteFilePath())) {
                m_impl->loadLibrary(fileInfo);
            }
        }
    }
}

GlobalObjectFactory* GlobalObjectFactory::instance(){
    return g_defaultObjectFactory;
}




//
}