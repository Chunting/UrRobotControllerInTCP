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

class GlobalObjectFactory {
public:
    GlobalObjectFactory();
    ~GlobalObjectFactory();


    std::shared_ptr<AbstractObject> createObject(const std::string& factory, const std::string& type);

    void loadLibrarys(const QString& path);


    static GlobalObjectFactory* instance();
private:
    class GlobalObjectFactoryImpl;
    std::shared_ptr<GlobalObjectFactoryImpl> m_impl;
};

//
}

#endif //PROJECT_COBOTSYS_GLOBAL_OBJECT_FACTORY_H
