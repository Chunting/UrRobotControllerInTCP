//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URDEBUGGERWIDGET_H
#define PROJECT_URDEBUGGERWIDGET_H

#include <cobotsys_abstract_robot_driver.h>
#include <cobotsys_abstract_controller.h>

using namespace cobotsys;
class UrDebuggerWidget : public cobotsys::AbstractControllerWidget,
                         public RobotStatusObserver {
Q_OBJECT
public:
    UrDebuggerWidget();
    virtual ~UrDebuggerWidget();

    virtual bool start();
    virtual void pause();
    virtual void stop();

    virtual bool setup(const QString& configFilePath);

public:
    virtual void onMoveFinish(uint32_t moveId);
    virtual void onJointStatusUpdate(const std::vector<double>& jointPose);
    virtual void onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot);
    virtual void onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot);

protected:
    std::shared_ptr<AbstractRobotDriver> m_robotDriver;
};


#endif //PROJECT_URDEBUGGERWIDGET_H
