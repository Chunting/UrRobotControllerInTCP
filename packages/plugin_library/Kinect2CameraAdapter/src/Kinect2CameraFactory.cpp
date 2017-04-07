//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <libfreenect2/logger.h>
#include <extra2.h>
#include <cobotsys_abstract_factory_macro.h>
#include "Kinect2Camera.h"

//    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

COBOTSYS_FACTORY_BEGIN(Kinect2CameraFactory)
        COBOTSYS_FACTORY_EXPORT(Kinect2Camera)
COBOTSYS_FACTORY_END(Kinect2CameraFactory, "1.0")