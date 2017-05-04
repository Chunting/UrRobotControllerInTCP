//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <vector>
#include <map>
#include <string>
#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_factory_macro.h>
#include "Ur10JointFilter.h"

COBOTSYS_FACTORY_BEGIN(Ur10FilterFactory)
COBOTSYS_FACTORY_EXPORT(Ur10JointFilter)
COBOTSYS_FACTORY_END(Ur10FilterFactory, "1.0")