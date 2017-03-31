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

/**
 * @note ObjectGroup是一个辅助的类，用于从JSON配置里创建对象
 *       有些时候同一个对象如果有多个接口，可以让一个对象来表示多个，
 *       可以避免重复创建

  下面是JSON格式的例子，objectGroup是ObjectGroup对象所需要的数据项
  它的值是一个数组
  @arg objectId  要创建的对象的ID，ID必须唯一，但是Key可以相同，即
                 两个ID可以使用同一个具体对象。
  @arg objectKey 对象的唯一Key
  @arg factory   创建对象所在的工厂库
  @arg type      对象类型
  @arg config    配置文件路径，可以是相对路径，会基本FileFinder来查找文件

  如果objectKey已经创建过了，factory与type会被忽略。
{
  "objectGroup": [
    {
      "objectId": "camera",
      "objectKey": "cameraWidget",
      "factory": "Kinect2CameraFactory, Ver 1.0",
      "type": "Kinect2",
      "config": "camera_param.yaml"
    },
    {
      "objectId": "view",
      "objectKey": "cameraWidget"
    }
  ]
}
 *
 * 对像可以有相同的
 *
 */
class ObjectGroup {
public:
    ObjectGroup();
    ~ObjectGroup();


    /**
     * 依据JSON文件配置来动态创建对象，如果创建对象里有setup所需要的config，会自动的加载config并调用
     * setup函数。如果setup函数失败，会返回false并释放所有对象。
     * @param jsonConfig JSON文件路径
     * @return
     */
    bool init(const QJsonObject& jsonConfig);


    /**
     *
     * @param objectId 获取objectId所对就的对象。
     * @return 如果有返回对象指针，如果没有，返回nullptr
     */
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
            pInfo = nullptr;
            pObject = nullptr;
        }
    };

    std::map<std::string, std::string> m_idKeys;
    std::map<std::string, std::shared_ptr<ObjectInfo> > m_objs;
};

/**
 * 正常情况下，这个对象在main函数里初始化并创建一个基本对象。
 * 例如：

int main(int argc, char** argv){
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys("../lib/plugins");

    ........do something.....

    return a.exec();
}
 * 以上是这个对象的正确用法。
 */
class GlobalObjectFactory {
public:
    GlobalObjectFactory();
    ~GlobalObjectFactory();

    /**
     * 动态的创建一个由shared_ptr所管理的对象，这个对象一定从AbstractObject继承。
     * @param factory 工厂shared library类型
     * @param type    要创建的对象类型
     * @return
     */
    std::shared_ptr<AbstractObject> createObject(const std::string& factory, const std::string& type);
    std::shared_ptr<AbstractObject> createObject(const QString& factory, const QString& type);
    std::shared_ptr<AbstractObject> createObject(const char* factory, const char* type);

    /**
     * 从指定路加载动态库，Windows下是DLL，Linux下是so
     * @param path
     */
    void loadLibrarys(const QString& path);

    /**
     * 加载当前进程里的AbstractObjectFactory。
     * 这个函数其实是为ROS准备的。ROS不能通过shared object加载。会导致退出的时候异常。
     * @param factory
     */
    void addExtendLibrary(std::shared_ptr<AbstractObjectFactory> factory);

    /**
     * 获取当前已经管理的所有动态库的名称
     * @return
     */
    std::vector<std::string> getFactoryNames() const;

    /**
     * 获取对应动态库可以创建的所有对象类型的名称。
     * @param factoryName
     * @return
     */
    std::vector<std::string> getFactorySupportedNames(const std::string& factoryName) const;

    /**
     * 获取当前对象(GlobalObjectFactory)的实例。
     * @return
     */
    static GlobalObjectFactory* instance();
private:
    class GlobalObjectFactoryImpl;
    std::shared_ptr<GlobalObjectFactoryImpl> m_impl;
};

//
}

#endif //PROJECT_COBOTSYS_GLOBAL_OBJECT_FACTORY_H
