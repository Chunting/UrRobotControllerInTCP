//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_GRIPPER_H
#define PROJECT_COBOTSYS_ABSTRACT_GRIPPER_H

#include "cobotsys_abstract_object.h"

namespace cobotsys {

enum class GripperPosition {
    Open = 0,
    Close = 1000000,
};


class GripperStatusObserver {
public:
    GripperStatusObserver();
    virtual ~GripperStatusObserver();

    /**
     * AbstractGripper.setGripperPosition操作完成后，就会调用此函数
     * @param from 原来的位置
     * @param to 目的位置
     */
    virtual void onMoveFinish(int from, int to) = 0;
};

class AbstractGripper : public AbstractObject {
public:
    AbstractGripper();
    virtual ~AbstractGripper();

    /**
     * 设置运动速度, 返回实际的速度值
     * @param cm_per_sec
     * @return
     */
    virtual double setSpeed(double cm_per_sec) = 0;

    /**
     * 设置如果碰到目标物体，保持的力F是多大，单位，牛顿
     * @param newton
     * @return 实际可以达到的值
     */
    virtual double setHoldForce(double newton) = 0;

    /**
     *
     * @param position 除了Open与Close外，其他的位置也是
     * @return 失败表示不可到达的位置
     */
    virtual bool setGripperPosition(int position) = 0;
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_GRIPPER_H
