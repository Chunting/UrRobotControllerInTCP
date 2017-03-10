//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H

#include "cobotsys_abstract_object.h"
#include <opencv/cv.h>
#include <QtCore/QIODevice>


namespace cobotsys {
class AbstractRobotDriver;
class RobotStatusObserver {
public:
    RobotStatusObserver();
    virtual ~RobotStatusObserver();

    virtual void onMoveFinish(uint32_t moveId) = 0;
    virtual void onJointStatusUpdate() = 0;
    virtual void onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot) = 0;
    virtual void onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot) = 0;
};
}


namespace cobotsys {

enum class RobotIoStatus {
    On,
    Off,
};

enum class RobotIoPort {
    Port_1 = 1UL << 1,
    Port_2 = 1UL << 2,
    Port_3 = 1UL << 3,
    Port_4 = 1UL << 4,
    Port_5 = 1UL << 5,
    Port_6 = 1UL << 6,
    Port_7 = 1UL << 7,
    Port_8 = 1UL << 8,
};

Q_DECLARE_FLAGS(RobotIoPorts, RobotIoPort);

class AbstractRobotDriver : public AbstractObject {
public:
    AbstractRobotDriver();
    virtual ~AbstractRobotDriver();

    virtual void move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& normal) = 0;
    virtual void digitIoControl(RobotIoPorts ioPorts, RobotIoStatus ioStatus) = 0;

    virtual void attach(std::shared_ptr<RobotStatusObserver> observer) = 0;
    virtual void directControl(const std::string& command) = 0;

    virtual QIODevice* getExtraIoDevice(int devicdId) = 0;

    virtual bool getMoveData(uint32_t moveId, cv::Point3d& pos, cv::Vec3d& normal) = 0;
    virtual void clearMoveDataHistory() = 0;

    virtual void connect(const std::string& addrConfig) = 0;
    virtual void setup(const std::string& config) = 0;

    virtual void start() = 0;
    virtual void pause() = 0;

public:
    static uint32_t generateMoveId();
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_ROBOT_DRIVER_H
