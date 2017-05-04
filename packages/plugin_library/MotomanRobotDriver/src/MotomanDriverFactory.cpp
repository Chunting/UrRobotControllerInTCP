//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <cobotsys_abstract_object_factory.h>
#include "MotomanDriver.h"
#include <extra2.h>
#include <cobotsys_abstract_factory_macro.h>

COBOTSYS_FACTORY_BEGIN(MotomanDriverFactory)
        COBOTSYS_FACTORY_EXPORT(MotomanDriver)
COBOTSYS_FACTORY_END(MotomanDriverFactory, "1.0")
