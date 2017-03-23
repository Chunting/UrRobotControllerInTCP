//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H

#include "cobotsys_abstract_object.h"
#include "cobotsys_abstract_digit_io_driver.h"
#include <opencv/cv.h>
#include <QtCore/QIODevice>


namespace cobotsys {
class AbstractRobotDriver;
class RobotStatusObserver {
public:
    RobotStatusObserver();
    virtual ~RobotStatusObserver();

    virtual void onMoveFinish(uint32_t moveId) = 0;
    virtual void onJointStatusUpdate(const std::vector<double>& jointPose) = 0;
    virtual void onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot) = 0;
    virtual void onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot) = 0;
};
}


namespace cobotsys {
class AbstractRobotDriver : public AbstractObject {
public:
    AbstractRobotDriver();
    virtual ~AbstractRobotDriver();

    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal) = 0;
    virtual void move(uint32_t moveId, const std::vector<double>& jointPos) = 0;

    /**
     * 向驱动注测观察者，一般是在setup阶段就完成的应用。
     * @note attach方法不可以在运行过程中动态添加，因为不是所有的实现都实现了这种机制。
     * @param observer
     */
    virtual void attach(std::shared_ptr<RobotStatusObserver> observer) = 0;
    virtual void directControl(const std::string& command) = 0;

    virtual std::shared_ptr<QIODevice> getSerialIoDevice(int devicdId = 0) = 0;
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0) = 0;

    virtual bool getMoveData(uint32_t moveId, std::vector<double>& moveData, int& moveDataType) = 0;
    virtual void clearMoveDataHistory() = 0;

    virtual bool setup(const QString& configFilePath);
    virtual bool start() = 0;
    virtual void pause() = 0;

    virtual bool isReady() const = 0;

public:
    static uint32_t generateMoveId();
};
}

typedef std::shared_ptr<cobotsys::AbstractRobotDriver> AbstractRobotDriverPtr;

#endif //PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
