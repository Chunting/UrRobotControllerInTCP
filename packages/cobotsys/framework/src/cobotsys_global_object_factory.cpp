//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonArray>
#include <include/extra2.h>
#include <include/cobotsys_file_finder.h>
#include "cobotsys_global_object_factory.h"

#define OBJECT_FACTORY_SYMBOL "getAbstractObjectFactoryInstance"

typedef void* (* getAbstractObjectFactoryInstance)();

namespace cobotsys {
static GlobalObjectFactory* g_defaultObjectFactory = nullptr;


class GlobalObjectFactory::GlobalObjectFactoryImpl {
public:
    std::map<std::string, std::shared_ptr<AbstractObjectFactory> > objectFactoryMap;

    std::map<std::string, std::vector<std::string> > factorySupportTypes;


    bool hasFactory(const std::string& factoryType) const{
        return (objectFactoryMap.find(factoryType) != objectFactoryMap.end());
    }

    bool hasFactory(std::shared_ptr<AbstractObjectFactory>& factory){
        return hasFactory(factory->getFactoryType());
    }

    void appendFactory(std::shared_ptr<AbstractObjectFactory>& factory){
        objectFactoryMap[factory->getFactoryType()] = factory;
    }

    void dumpLibraryInfo(std::shared_ptr<AbstractObjectFactory>& shrFactory, const QFileInfo& fileInfo){
        auto tlist = shrFactory->getSupportTypes();

        factorySupportTypes[shrFactory->getFactoryType()] = tlist;

        std::stringstream oss;
        oss << "{";
        for (size_t i = 0; i < tlist.size(); i++) {
            oss << tlist[i];
            if (i + 1 < tlist.size()) oss << ", ";
        }
        oss << "}";

        if (fileInfo.fileName().isEmpty()) {
            COBOT_LOG.message("Extend")
                    << std::setw(32) << "ROS Extend" << ": "
                    << shrFactory->getFactoryType()
                    << ", " << oss.str();
        } else {
            COBOT_LOG.message("Plugin")
                    << std::setw(32) << fileInfo.fileName() << ": "
                    << shrFactory->getFactoryType()
                    << ", " << oss.str();
        }
    }

    void loadLibrary(const QFileInfo& fileInfo){
        auto fAddr = QLibrary::resolve(fileInfo.absoluteFilePath(), OBJECT_FACTORY_SYMBOL);
        auto qFunc = (getAbstractObjectFactoryInstance) fAddr;
        auto libnm = fileInfo.baseName().toStdString();

        if (qFunc) {
            auto rawFactory = static_cast<AbstractObjectFactory*>(qFunc());
            auto fftype = rawFactory->getFactoryType();
            auto shrFactory = rawFactory->shared_from_this();
            auto factoryType = shrFactory->getFactoryType();

            if (hasFactory(shrFactory)) {
            } else {
                appendFactory(shrFactory);
                dumpLibraryInfo(shrFactory, fileInfo);
            }
        } else {
            COBOT_LOG.warning() << "Unable to get Entry for: " << libnm;
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
    INFO_DESTRUCTOR(this);
}

std::shared_ptr<AbstractObject> GlobalObjectFactory::createObject(const std::string& factory, const std::string& type){
    return m_impl->createObject(factory, type);
}

std::shared_ptr<AbstractObject> GlobalObjectFactory::createObject(const QString& factory, const QString& type){
    return createObject((std::string) factory.toLocal8Bit().constData(), (std::string) type.toLocal8Bit().constData());
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

std::shared_ptr<AbstractObject> GlobalObjectFactory::createObject(const char* factory, const char* type){
    return createObject(std::string(factory), std::string(type));
}

void GlobalObjectFactory::addExtendLibrary(std::shared_ptr<AbstractObjectFactory> factory){
    if (m_impl->hasFactory(factory)) {
    } else {
        m_impl->appendFactory(factory);
        m_impl->dumpLibraryInfo(factory, QFileInfo());
    }
}

std::vector<std::string> GlobalObjectFactory::getFactoryNames() const{
    std::vector<std::string> rNames;
    for (const auto& iter : m_impl->factorySupportTypes) {
        rNames.push_back(iter.first);
    }
    return rNames;
}

std::vector<std::string> GlobalObjectFactory::getFactorySupportedNames(const std::string& factoryName) const{
    const auto iter = m_impl->factorySupportTypes.find(factoryName);
    if (iter != m_impl->factorySupportTypes.end()) {
        return iter->second;
    }
    return std::vector<std::string>();
}

void GlobalObjectFactory::loadLibrarys(const std::string& path){
    loadLibrarys(QString::fromLocal8Bit(path.c_str()));
}

void GlobalObjectFactory::loadLibrarys(){
    loadLibrarys(FileFinder::getPreDefPath(FileFinder::Plugin));
}
}

namespace cobotsys {
ObjectGroup::ObjectGroup(){
}

ObjectGroup::~ObjectGroup(){
    m_idKeys.clear();
    m_objs.clear();
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

        /// @note 设置目标对象的基本参数数据。从文件配置来设置。
        if (objConfig.contains("config")) {
            if (!pObject->setup(objConfig["config"].toString())) {
                COBOT_LOG.warning() << "Fail to setup: " << factory << ", " << type;
                return false;
            }
        }

        /// @note 保存对象到map里。
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