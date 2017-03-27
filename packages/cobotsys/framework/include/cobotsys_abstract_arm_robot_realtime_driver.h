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
struct ArmRobotStatus {
    std::vector<double> q_target; // target joint position
    std::vector<double> qd_target; // target joint velocity
    std::vector<double> qdd_target; // target joint acceleration

    std::vector<double> q_actual; // actual joint position
    std::vector<double> qd_actual; // actual joint velocity
    std::vector<double> qdd_actual; // acutal joint acceleration
};
typedef std::shared_ptr<ArmRobotStatus> ArmRobotStatusPtr;

class ArmRobotRealTimeStatusObserver {
public:
    ArmRobotRealTimeStatusObserver();
    virtual ~ArmRobotRealTimeStatusObserver();

    virtual void onArmRobotConnect() = 0;
    virtual void onArmRobotDisconnect() = 0;
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) = 0;
};

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
     * @param deviceId
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
     * 启动机器人控制。即Move命令现在有效
     * @return
     */
    virtual bool start() = 0;
    /**
     * 停止实时机器人控制
     */
    virtual void stop() = 0;
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_ARM_ROBOT_REALTIME_DRIVER_H
