//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_GLOBAL_OBJECT_FACTORY_H
#define PROJECT_COBOTSYS_GLOBAL_OBJECT_FACTORY_H

#include <memory>
#include <map>
#include <QLibrary>
#include <QDir>
#include <QString>
#include <cobotsys_abstract_object.h>
#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_logger.h>
#include <cobotsys_qt.h>

namespace cobotsys {

class ObjectGroup {
public:
    ObjectGroup();
    ~ObjectGroup();


    bool init(const QJsonObject& jsonConfig);


    std::shared_ptr<AbstractObject> getObject(const std::string& objectId);
protected:
    bool _initImpl(const QJsonObject& jsonConfig);
protected:
    struct ObjectSourceInfo {
        std::string factory;
        std::string type;
    };

    struct ObjectInfo {
        std::shared_ptr<ObjectSourceInfo> pInfo;
        std::shared_ptr<AbstractObject> pObject;

        ~ObjectInfo(){
            if (pObject)
                pObject->resetAllSharedObject();
            pInfo = nullptr;
            pObject = nullptr;
        }
    };

    std::map<std::string, std::string> m_idKeys;
    std::map<std::string, std::shared_ptr<ObjectInfo> > m_objs;
};


class GlobalObjectFactory {
public:
    GlobalObjectFactory();
    ~GlobalObjectFactory();


    std::shared_ptr<AbstractObject> createObject(const std::string& factory, const std::string& type);
    std::shared_ptr<AbstractObject> createObject(const QString& factory, const QString& type);
    std::shared_ptr<AbstractObject> createObject(const char* factory, const char* type);

    void loadLibrarys(const QString& path);

    static GlobalObjectFactory* instance();
private:
    class GlobalObjectFactoryImpl;
    std::shared_ptr<GlobalObjectFactoryImpl> m_impl;
};

//
}

#endif //PROJECT_COBOTSYS_GLOBAL_OBJECT_FACTORY_H
