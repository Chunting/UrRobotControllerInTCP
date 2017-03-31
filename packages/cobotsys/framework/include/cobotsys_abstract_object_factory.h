//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H
#define PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H

#include <vector>
#include <string>
#include <memory>
#include <cobotsys_abstract_object.h>


namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 *
 * @defgroup factory
 * @brief 软件框架相关
 * @{
 */

/**
 *
 * @brief cobotsys的简单对工厂
 * 所有的动态库里，都要实现这个类
 * 即继承这个类后。
 *
 * 因为 GlobalObjectFactory 是通过这个类来创建具体的对象的。
 */
class AbstractObjectFactory : public std::enable_shared_from_this<AbstractObjectFactory> {
public:
    AbstractObjectFactory();
    virtual ~AbstractObjectFactory();

    /**
     * 返回支持的类型，以 vector <std::string> 的形式
     */
    virtual std::vector<std::string> getSupportTypes() = 0;
    virtual std::string getFactoryType() = 0; ///< 返回这个工厂类的简易信息

    /**
     * 创建指定的动态对象
     * @param[in] type 要创建的对象类型(字符串描述)
     * @return 返回创建的对象
     */
    virtual std::shared_ptr<AbstractObject> createObject(const std::string& type) = 0;
};
/**
 * @}
 * @}
 */
}

#endif //PROJECT_COBOTSYS_ABSTRACT_OBJECT_FACTORY_H
