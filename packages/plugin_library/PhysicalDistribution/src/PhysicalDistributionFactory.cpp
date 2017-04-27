//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>

#include "cobotsys_abstract_factory_macro.h"
#include "PhysicalDistributionController.h"
#include "RobotStatusViewer.h"

COBOTSYS_FACTORY_BEGIN(PhysicalDistributionFactory)
        COBOTSYS_FACTORY_EXPORT(PhysicalDistributionController)
        COBOTSYS_FACTORY_EXPORT(RobotStatusViewer)
COBOTSYS_FACTORY_END(PhysicalDistributionFactory, "1.0")