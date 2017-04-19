//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include <RobotXyz/RobotXyzWidget.h>
#include <ArmRobotManipulator/ArmRobotManipulator.h>
#include <SimpleWidgetViewer/SimpleWidgetViewer.h>
#include <DragApp/DragAppWidget.h>
//TODO 左斜线在Windows系统中可编译，但是不可识别。导致IDE找不到h文件，是否可以用""的方式来引用头文件。
#include "cobotsys_abstract_factory_macro.h"
COBOTSYS_FACTORY_BEGIN(SimpleUiFactory)
        COBOTSYS_FACTORY_EXPORT(RobotXyzWidget)
        COBOTSYS_FACTORY_EXPORT(ArmRobotManipulator)
        COBOTSYS_FACTORY_EXPORT(SimpleWidgetViewer)
		COBOTSYS_FACTORY_EXPORT(DragAppWidget)
COBOTSYS_FACTORY_END(SimpleUiFactory, "1.0")