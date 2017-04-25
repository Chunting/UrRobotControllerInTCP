//
// Created by 潘绪洋 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "cobotsys_abstract_factory_macro.h"
#include "SampleVisionDetector.h"

COBOTSYS_FACTORY_BEGIN(SampleVisionDetectorFactory)
        COBOTSYS_FACTORY_EXPORT(SampleVisionDetector)
COBOTSYS_FACTORY_END(SampleVisionDetectorFactory, "1.0")