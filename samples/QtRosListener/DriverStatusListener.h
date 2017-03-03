//
// Created by 潘绪洋 on 17-3-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRIVERSTATUSLISTENER_H
#define PROJECT_DRIVERSTATUSLISTENER_H

#include <cobotsys_qt_ros_node.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class DriverStatusListener : public cobotsys::QtRosNode {
public:
    DriverStatusListener(int argc, char** argv);
    virtual ~DriverStatusListener();

    void run();
    void ros_comms_init();

private:
    void robotStatusUpdateCallback(const std_msgs::String::ConstPtr& msg);
    ros::Subscriber robot_status_subscriber;
};


#endif //PROJECT_DRIVERSTATUSLISTENER_H
