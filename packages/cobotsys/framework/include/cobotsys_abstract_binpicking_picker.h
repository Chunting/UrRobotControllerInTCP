//
// Created by 潘绪洋 on 17-3-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PICKER_H
#define PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PICKER_H


#include "cobotsys_abstract_object.h"
#include "cobotsys_abstract_arm_robot_move_driver.h"
#include "cobotsys_data_types.h"

namespace cobotsys {
namespace binpicking {
class AbstractBinpickingPicker : public AbstractObject {
public:
    AbstractBinpickingPicker();
    virtual ~AbstractBinpickingPicker();

    virtual bool pickObject(const BinObjGrabPose& binObjGrabPose) = 0;
    virtual void setRobotDriver(std::shared_ptr<AbstractArmRobotMoveDriver> robotDriver) = 0;
    virtual void setDigitIoDriver(std::shared_ptr<AbstractDigitIoDriver> digitIoDriver) = 0;
};
}
}


#endif //PROJECT_COBOTSYS_ABSTRACT_BINPICKING_PICKER_H
