//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_abstract_digit_io_driver.h"
#include "cobotsys_abstract_kinematic_solver.h"
#include <opencv2/opencv.hpp>
#include <QIODevice>
#include <cobotsys_logger.h>


namespace cobotsys {

enum class MoveResult {
    Success,
    InvalidMoveTarget,
    Cancled,
};

class ArmRobotMoveStatusObserver {
public:
    ArmRobotMoveStatusObserver();
    virtual ~ArmRobotMoveStatusObserver();

    /**
     * AbstractArmRobotMoveDriver::move() 函数操作完成后，会调用此函数
     * @param moveId
     */
    virtual void onMoveFinish(uint32_t moveId, MoveResult moveResult) = 0;
};
}


namespace cobotsys {

struct RobotWaypoint {
    uint32_t moveId;
    double desireVelocity;
    cv::Point3d position;
    cv::Vec3d rpy;

    std::vector<double> toArray() const {
        return {
                position.x, position.y, position.z,
                rpy[0], rpy[1], rpy[2]
        };
    }

    static RobotWaypoint fromArray(const std::vector<double>& posi, double vel = 1.0) {
        RobotWaypoint robotWaypoint;
        robotWaypoint.moveId = 0;
        robotWaypoint.desireVelocity = vel;
        if (posi.size() >= 6) {
            robotWaypoint.position = {posi[0], posi[1], posi[2]};
            robotWaypoint.rpy = {posi[3], posi[4], posi[5]};
        } else if (posi.size() != 0) {
            COBOT_LOG.warning() << "Invalid POSE Size!";
        }
        return robotWaypoint;
    }
};


class AbstractArmRobotRealTimeDriver;
class AbstractArmRobotMoveDriver : public AbstractObject {
public:
    AbstractArmRobotMoveDriver();
    virtual ~AbstractArmRobotMoveDriver();

    /**
     * 控制机器人做一次位置移动，移动速度是尽可能的快。
     * 如果还有更多的控制选项，可以从参数里来做控制，即 setup() 函数设置，通过配置文件。
     * 比如， 最快速度
     * @param[in] moveId 本次移动的ID，由 generateMoveId() 生成。
     * @param[in] pos 相对于基坐标
     * @param[in] rpy roll, pitch, yaw相对于基坐标，按roll, pitch, yaw顺序
     * @retval true 目标位置可以到达
     * @retval false 目标位置不可到达
     */
    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy) = 0;

    /**
     *
     * @param[in] moveId 本次移动的ID，由 generateMoveId() 生成。
     * @param[in] waypoints 一次性的给出关键点。然后规划路径
     * @retval true 目标位置可以到达
     * @retval false 目标位置不可到达
     */
    virtual bool move(const std::vector<RobotWaypoint>& waypoints) = 0;

    /**
     * 当一次 move() 操作结束后， 即会调用对应的 onMoveFinish() 函数
     * @param[in] observer 观察者对象
     */
    virtual void attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer) = 0;


    /**
     * 设置底层驱动
     */
    virtual void setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver) = 0;

    /**
     * 设置逆运动学对象
     */
    virtual void setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver) = 0;

    virtual std::shared_ptr<AbstractKinematicSolver>& getKinematicSolver() = 0;

    virtual bool start() = 0;
    virtual void clearAll() = 0; /// 清除当前，以及所有的移动目标

public:
    /**
     * 用于生成 move() 操作所需要的 moveId
     * thread safe
     * @return 返回一个 moveId 数值
     */
    static uint32_t generateMoveId();
};


class AbstractSyncMover : public AbstractObject {
public:
    AbstractSyncMover();
    virtual ~AbstractSyncMover();

    /**
     * 把所有的点都移动完成后才返回, 是基于AbstractArmRobotMoveDriver的一种基本封装。
     * @param movePoints
     */
    virtual bool move(const std::vector<RobotWaypoint>& movePoints) = 0;
    virtual void setArmRobotMover(const std::shared_ptr<AbstractArmRobotMoveDriver>& robotMover) = 0;
};
}

typedef std::shared_ptr<cobotsys::AbstractArmRobotMoveDriver> AbstractArmRobotMoveDriverPtr;

#endif //PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
