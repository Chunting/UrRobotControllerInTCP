//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_UR3DRIVERSTATUSREPORTER_H
#define PROJECT_UR3DRIVERSTATUSREPORTER_H

#include <cobotsys_qt_ros_node.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class Ur3DriverStatusReporter : public cobotsys::QtRosNode {
Q_OBJECT
public:
    Ur3DriverStatusReporter(int argc, char** argv, const char* name);
    virtual ~Ur3DriverStatusReporter();

    void run();
    void ros_comms_init();

Q_SIGNALS:
    void robotControlStatusUpdated(const QString&);

private:
    void robotControlStatusCallback(const std_msgs::String::ConstPtr& msg);
    ros::Subscriber robot_status_subscriber;
};


#endif //PROJECT_UR3DRIVERSTATUSREPORTER_H
