//
// Created by 潘绪洋 on 17-4-26.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>

#include "cobotsys_abstract_factory_macro.h"
#include "UrMover.h"
#include "SyncMover.h"
#include "InterCubicMover.h"

COBOTSYS_FACTORY_BEGIN(UrMoverFactory)
        COBOTSYS_FACTORY_EXPORT(UrMover)
        COBOTSYS_FACTORY_EXPORT(SyncMover)
        COBOTSYS_FACTORY_EXPORT(InterCubicMover)
COBOTSYS_FACTORY_END(UrMoverFactory, "1.0")
