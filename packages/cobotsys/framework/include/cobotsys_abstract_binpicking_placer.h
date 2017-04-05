//
// Created by 潘绪洋 on 17-3-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PLACER_H
#define PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PLACER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_abstract_arm_robot_move_driver.h"

namespace cobotsys {
class AbstractBinpickingPlacer : public AbstractObject {
public:
    AbstractBinpickingPlacer();
    virtual ~AbstractBinpickingPlacer();

    virtual void placeObject() = 0;
    virtual void setRobotDriver(std::shared_ptr<AbstractArmRobotMoveDriver> robotDriver) = 0;
    virtual void setDigitIoDriver(std::shared_ptr<AbstractDigitIoDriver> digitIoDriver) = 0;
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PLACER_H
