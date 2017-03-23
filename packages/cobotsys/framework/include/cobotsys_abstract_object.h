//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_OBJECT_H
#define PROJECT_COBOTSYS_ABSTRACT_OBJECT_H

#include <memory>
#include <QString>

namespace cobotsys {


class AbstractObject : public std::enable_shared_from_this<AbstractObject> {
public:
    AbstractObject();
    virtual ~ AbstractObject();

    /**
     * 如果对象有初始化的配置文件，那么在通过创建配置文件的时候，文件的路径地址会通过setup参数传入。
     * 同时，正常情况下。如果配置文件不对，无法满足设置要求，直接返回false就好。
     * @param configFilePath 当前对象所需要的配置文件的路径
     * @return
     */
    virtual bool setup(const QString& configFilePath);
    /**
     * 创建这个函数是因为
     * A - shared
     * B - shared
     * A has B(shared)
     * B.attach(A)
     *
     * C has A and B
     * C freed
     * A and B 仍然拥有彼此。无法释放。
     */
    virtual void resetAllSharedObject();
};


//
}

typedef std::shared_ptr<cobotsys::AbstractObject> AbstractObjectPtr;

#endif //PROJECT_COBOTSYS_ABSTRACT_OBJECT_H
