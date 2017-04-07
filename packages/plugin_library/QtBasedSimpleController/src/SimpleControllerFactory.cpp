//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include <cobotsys_abstract_factory_macro.h>
#include "CameraColorViewer.h"
#include "CameraColorViewer2.h"
#include "EmptyWidget.h"


COBOTSYS_FACTORY_BEGIN(SimpleControllerFactory)
        COBOTSYS_FACTORY_EXPORT(CameraColorViewer)
        COBOTSYS_FACTORY_EXPORT(CameraColorViewer2)
        COBOTSYS_FACTORY_EXPORT(EmptyWidget)
COBOTSYS_FACTORY_END(SimpleControllerFactory, "1.0")