//
// Created by 潘绪洋 on 17-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "cobotsys_abstract_factory_macro.h"
#include "OpenCVCamera.h"

COBOTSYS_FACTORY_BEGIN(OpenCVCameraFactory)
        COBOTSYS_FACTORY_EXPORT(OpenCVCamera)
COBOTSYS_FACTORY_END(OpenCVCameraFactory, "1.0")