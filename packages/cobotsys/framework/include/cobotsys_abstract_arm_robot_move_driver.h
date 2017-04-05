//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_abstract_digit_io_driver.h"
#include <opencv2/opencv.hpp>
#include <QIODevice>


namespace cobotsys {
class ArmRobotMoveStatusObserver {
public:
    ArmRobotMoveStatusObserver();
    virtual ~ArmRobotMoveStatusObserver();

    /**
     * AbstractArmRobotMoveDriver::move() 函数操作完成后，会调用此函数
     * @param moveId
     */
    virtual void onMoveFinish(uint32_t moveId) = 0;
};
}


namespace cobotsys {
class AbstractArmRobotMoveDriver : public AbstractObject {
public:
    AbstractArmRobotMoveDriver();
    virtual ~AbstractArmRobotMoveDriver();

    /**
     * 控制机器人做一次位置移动，移动速度是尽可能的快。
     * 如果还有更多的控制选项，可以从参数里来做控制，即 setup() 函数设置，通过配置文件。
     * 比如， 最快速度
     * @param[in] moveId 本次移动的ID，由 generateMoveId() 生成。
     * @param[in] pos 末段TCP的相对于基坐标的位置。
     * @param[in] rpy roll, pitch, yaw末段TCP的姿态，按roll, pitch, yaw顺序
     * @retval true 目标位置可以到达
     * @retval false 目标位置不可到达
     */
    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy) = 0;

    /**
     * 当一次 move() 操作结束后， 即会调用对应的 onMoveFinish() 函数
     * @param[in] observer 观察者对象
     */
    virtual void attach(std::shared_ptr<ArmRobotMoveStatusObserver> observer) = 0;

public:
    /**
     * 用于生成 move() 操作所需要的 moveId
     * thread safe
     * @return 返回一个 moveId 数值
     */
    static uint32_t generateMoveId();
};
}

typedef std::shared_ptr<cobotsys::AbstractArmRobotMoveDriver> AbstractArmRobotMoveDriverPtr;

#endif //PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
