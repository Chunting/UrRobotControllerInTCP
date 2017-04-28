//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_WIDGET_H
#define PROJECT_COBOTSYS_ABSTRACT_WIDGET_H

#include <QWidget>
#include <cobotsys_abstract_object.h>
#include "cobotsys_abstract_arm_robot_realtime_driver.h"
#include "cobotsys_abstract_arm_robot_move_driver.h"

namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 */

/**
 * @brief 一般UI接口，基于Widget
 *
 * 创建这个类只是为了把在其他动态库里的Widget动态的创建出来。这个类本身没有什么。
 */
class AbstractWidget : public QWidget, public AbstractObject {
public:
    AbstractWidget();
    virtual ~AbstractWidget();
};

class AbstractManipulator {
public:
    AbstractManipulator();
    virtual ~AbstractManipulator();

    virtual void setRobotMover(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& robot,
                               const std::shared_ptr<AbstractArmRobotMoveDriver>& mover) = 0;
};
/**
 * @}
 */
}

#endif //PROJECT_COBOTSYS_ABSTRACT_WIDGET_H
