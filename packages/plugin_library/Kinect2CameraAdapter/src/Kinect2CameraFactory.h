//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_KINECT2CAMERAFACTORY_H
#define PROJECT_KINECT2CAMERAFACTORY_H

#include <cobotsys_abstract_object_factory.h>

class Kinect2CameraFactory : public cobotsys::AbstractObjectFactory {

public:
    Kinect2CameraFactory();
    virtual ~Kinect2CameraFactory();

    virtual std::vector<std::string> getSupportTypes();
    virtual std::string getFactoryType();
    virtual std::shared_ptr<cobotsys::AbstractObject> createObject(const std::string& type);
};


#endif //PROJECT_KINECT2CAMERAFACTORY_H
