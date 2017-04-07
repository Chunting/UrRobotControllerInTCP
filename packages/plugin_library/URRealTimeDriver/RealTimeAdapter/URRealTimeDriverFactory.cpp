//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "URRealTimeDriver.h"
#include "cobotsys_abstract_factory_macro.h"

COBOTSYS_FACTORY_BEGIN(URRealTimeDriverFactory)
        COBOTSYS_FACTORY_EXPORT(URRealTimeDriver)
COBOTSYS_FACTORY_END(URRealTimeDriverFactory, "1.0")