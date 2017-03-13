//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonArray>
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
}

namespace cobotsys {
ObjectGroup::ObjectGroup(){
}

ObjectGroup::~ObjectGroup(){
}

bool ObjectGroup::init(const QJsonObject& jsonConfig){
    m_idKeys.clear();
    m_objs.clear();

    if (_initImpl(jsonConfig))
        return true;

    m_idKeys.clear();
    m_objs.clear();
    return false;
}

bool ObjectGroup::_initImpl(const QJsonObject& jsonConfig){
    if (GlobalObjectFactory::instance() == nullptr)
        return false;

    auto groupObjs = jsonConfig["objectGroup"].toArray();
    for (const auto& obj : groupObjs) {
        auto objConfig = obj.toObject();

        std::string objectId = objConfig["objectId"].toString().toLocal8Bit().constData();
        std::string objectKey = objConfig["objectKey"].toString().toLocal8Bit().constData();
        if (objectId.empty() || objectKey.empty()) {// 对象的ID与Key为空，不合法
            return false;
        }

        if (m_idKeys.find(objectId) != m_idKeys.end()) { // 有相同ID的对象，不合法
            return false;
        }
        m_idKeys[objectId] = objectKey;

        auto iter = m_objs.find(objectKey);
        if (iter != m_objs.end()) { // 目标对象已经存在
            continue;
        }

        std::string factory = objConfig["factory"].toString().toLocal8Bit().constData();
        std::string type = objConfig["type"].toString().toLocal8Bit().constData();

        auto pObject = GlobalObjectFactory::instance()->createObject(factory, type);
        if (pObject == nullptr) {
            COBOT_LOG.warning() << "Fail to create: " << factory << ", " << type;
            return false;
        }

        auto srcInfo = std::make_shared<ObjectSourceInfo>();
        srcInfo->factory = factory;
        srcInfo->type = type;
        auto objInfo = std::make_shared<ObjectInfo>();
        objInfo->pInfo = srcInfo;
        objInfo->pObject = pObject;
        m_objs[objectKey] = objInfo;
    }
    return true;
}

std::shared_ptr<AbstractObject> ObjectGroup::getObject(const std::string& objectId){
    auto kIter = m_idKeys.find(objectId);
    if (kIter != m_idKeys.end()) {
        auto oIter = m_objs.find(kIter->second);
        if (oIter != m_objs.end()) {
            return oIter->second->pObject;
        }
    }
    return nullptr;
}
}