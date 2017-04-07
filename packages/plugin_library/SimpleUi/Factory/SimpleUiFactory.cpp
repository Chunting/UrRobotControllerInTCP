//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include <RobotXyz/RobotXyzWidget.h>
#include <ArmRobotManipulator/ArmRobotManipulator.h>
#include <SimpleWidgetViewer/SimpleWidgetViewer.h>

#include "cobotsys_abstract_factory_macro.h"

COBOTSYS_FACTORY_BEGIN(SimpleUiFactory)
        COBOTSYS_FACTORY_EXPORT(RobotXyzWidget)
        COBOTSYS_FACTORY_EXPORT(ArmRobotManipulator)
        COBOTSYS_FACTORY_EXPORT(SimpleWidgetViewer)
COBOTSYS_FACTORY_END(SimpleUiFactory, "1.0")