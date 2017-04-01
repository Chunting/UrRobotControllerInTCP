//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_ARM_ROBOT_REALTIME_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_ARM_ROBOT_REALTIME_DRIVER_H

#include <vector>
#include <cobotsys_abstract_object.h>
#include <cobotsys_abstract_digit_io_driver.h>

namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 * @defgroup robot
 * @{
 *
 * @brief 机器人驱动接口
 */

/**
 * @brief 机器人（手臂）实时状态
 */
struct ArmRobotStatus {
    std::vector<double> q_target; ///< target joint position
    std::vector<double> qd_target; ///< target joint velocity
    std::vector<double> qdd_target; ///< target joint acceleration

    std::vector<double> q_actual; ///< actual joint position
    std::vector<double> qd_actual; ///< actual joint velocity
    std::vector<double> qdd_actual; ///< acutal joint acceleration
};

typedef std::shared_ptr<ArmRobotStatus> ArmRobotStatusPtr; ///<  ArmRobotStatus 智能指针


/**
 *
 * 用于通知观察者的回调事件函数
 */
class ArmRobotRealTimeStatusObserver {
public:
    ArmRobotRealTimeStatusObserver();
    virtual ~ArmRobotRealTimeStatusObserver();

    /**
     * 机器人成功启动
     */
    virtual void onArmRobotConnect() = 0;

    /**
     * 机器人丢失链接，启动失败...
     */
    virtual void onArmRobotDisconnect() = 0;

    /**
     * 机器人实时状态回调
     * @param ptrRobotStatus
     */
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) = 0;
};


/**
 *
 * 实时手臂，多关节机器人驱动接口，提供Joint关节脚驱动。
 * move()
 */
class AbstractArmRobotRealTimeDriver : public AbstractObject {
public:
    AbstractArmRobotRealTimeDriver();
    virtual ~AbstractArmRobotRealTimeDriver();

    /**
     * 这个是一个实时控制的接口，每次调用都会直接反应到当前控制周期内的效果
     * 同一个控制周期内，多次调用，以最后一次调用的值为准。
     * @param q 目标Joint位置
     */
    virtual void move(const std::vector<double>& q) = 0;

    /**
     * 获取机器人自带的IO接口控制
     * @param[in] deviceId
     * @return
     */
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0) = 0;

    /**
     * 向机器人注册观察者，当机器人事件发生后，会通过观察者接口API来通知所有已注册对象。
     * 顺序通知。
     * 已经实现线程安全
     * @param observer
     */
    virtual void attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer) = 0;

    /**
     * 启动机器人控制。仅仅只是发送命令过程没有问题。实际上需要
     * ArmRobotRealTimeStatusObserver::onArmRobotConnect 函数调用才是
     * 正常的启动了机器人。
     * @note 这个调用完成后，必然会触发观察事件。是否成功启动机器人，以回调事件为准
     * @retval true 机器人启动命令发送成功
     * @retval false 命令发送失败， 具体原因参见log.
     */
    virtual bool start() = 0;

    /**
     * 停止实时机器人控制
     */
    virtual void stop() = 0;

    /**
     * 返回机器人的实际IP，即 setup() 里设置的IP，可以从这个读取
     * @return 机器人当前的URL地址
     */
    virtual QString getRobotUrl() = 0;
};
/**
 * @}
 * @}
 */
}

#endif //PROJECT_COBOTSYS_ABSTRACT_ARM_ROBOT_REALTIME_DRIVER_H
