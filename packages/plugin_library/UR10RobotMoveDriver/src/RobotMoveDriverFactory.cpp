//
// Created by eleven on 17-4-14.
//
#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_factory_macro.h>
#include "UR10RobotMoveDriver.h"

COBOTSYS_FACTORY_BEGIN(RobotMoveDriverFactory)
        COBOTSYS_FACTORY_EXPORT(UR10RobotMoveDriver)
COBOTSYS_FACTORY_END(RobotMoveDriverFactory, "1.0")